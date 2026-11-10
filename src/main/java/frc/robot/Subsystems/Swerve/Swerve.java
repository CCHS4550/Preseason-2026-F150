package frc.robot.Subsystems.Swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.DriveConstants;
import frc.robot.Constants.RobotConstants.DriveConstants.ModuleOrientation;
import frc.robot.Subsystems.Swerve.Gyro.GyroIO;
import frc.robot.Subsystems.Swerve.Gyro.GyroIO.GyroIOInputs;
import frc.robot.Subsystems.Swerve.Module.ModuleIO;
import frc.robot.Subsystems.Swerve.Module.SwerveModule;
// import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
// import com.pathplanner.lib.util.swerve.SwerveSetpoint;

public class Swerve extends SubsystemBase {
	static final Lock odometryLock = new ReentrantLock();

	private final SwerveModule[] modules = new SwerveModule[4];
	private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

	private final GyroIO gyro;
	private GyroIOInputs gyroInputs = new GyroIOInputs();
	private Rotation2d gyroAngle = new Rotation2d();
	private final Alert gyroDCAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

	private CommandXboxController primaryController;

	private SwerveDriveKinematics kinematics 
		= new SwerveDriveKinematics(DriveConstants.MODULE_OFFSETS);

	
	private SwerveDrivePoseEstimator poseEstimator
		= new SwerveDrivePoseEstimator(kinematics, gyroAngle, lastModulePositions, new Pose2d());

	// private final SwerveSetpointGenerator setPointGenerator;
	// private SwerveSetpoint previousSetpoint; //maybe later

	private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();
		
	public enum WantedState
	{
		TELEOP_DRIVE,
		AUTO,
		IDLE
	}

	public enum SystemState
	{
		TELEOP_DRIVE,
		AUTO,
		IDLE,
	}

	private SystemState systemState = SystemState.IDLE;
	private WantedState wantedState = WantedState.IDLE;

	public Swerve(
		GyroIO gyro,
		ModuleIO flModuleIO,
      	ModuleIO frModuleIO,
      	ModuleIO blModuleIO,
      	ModuleIO brModuleIO,
		CommandXboxController primaryController
	) {
		this.gyro = gyro;
		modules[0] = new SwerveModule(flModuleIO, ModuleOrientation.FRONT_LEFT);
    	modules[1] = new SwerveModule(frModuleIO, ModuleOrientation.FRONT_RIGHT);
    	modules[2] = new SwerveModule(blModuleIO, ModuleOrientation.BACK_LEFT);
   	 	modules[3] = new SwerveModule(brModuleIO, ModuleOrientation.BACK_RIGHT);
		this.primaryController = primaryController;

		SparkOdometryThread.getInstance().start();
	}
	
	@Override
	public void periodic()
	{
		odometryLock.lock();
		gyro.updateInputs(gyroInputs);
		for(var module : modules)
		{
			module.periodic();
		}
		odometryLock.unlock();

		if (DriverStation.isDisabled()) 
		{
			stop();
		}

		double[] sampleTimestamps =
			modules[0].getOdometryTimestamps(); // get an array of timestamps from modules
		int sampleCount = sampleTimestamps.length; // number of samples to work through

		// loop for all samples of odometry information
		for (int i = 0; i < sampleCount; i++) {
			SwerveModulePosition[] modulePositions =
				new SwerveModulePosition[4]; // position of each module
			SwerveModulePosition[] moduleDeltas =
				new SwerveModulePosition[4]; // change in position of each module
			// loop through all modules in a given sample timestamp
			for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
				modulePositions[moduleIndex] =
					modules[moduleIndex].getOdometryPositions()[i]; // set the position at given timestamp

				moduleDeltas[moduleIndex] =
					new SwerveModulePosition(
						modulePositions[moduleIndex].distanceMeters
							- lastModulePositions[moduleIndex].distanceMeters,
						modulePositions[moduleIndex]
							.angle); // find difference in the module position of given timestamp vs
				// previous
				lastModulePositions[moduleIndex] =
					modulePositions[moduleIndex]; // set previous timestamp module position
			}

			// update gyro information
			if (gyroInputs.connected) {
				gyroAngle = gyroInputs.odometryYawPositions[i]; // update using our real gyro
			} else {
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				gyroAngle =
					gyroAngle.plus(new Rotation2d(twist.dtheta)); // update using robot odometry
			}

			// alert of gyro is disconnected
			gyroDCAlert.set(!gyroInputs.connected);

			poseEstimator.updateWithTime(
				sampleTimestamps[i],
				gyroAngle,
				modulePositions); // use poseEstimators inbuild function to update with our newest
			// odometry information, coordinated with timestamps
		}


		systemState = handleStateTransitions();
		applyStates();
	}
	//handle state transitions

	private void applyStates()
	{
		switch(systemState)
		{
			case AUTO:
				break;
			case TELEOP_DRIVE:
				setTargetChassisSpeeds();
				SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);
				for (int i = 0; i < 4; i++) {
					modules[i].runSwerveState(moduleStates[i]);
				}
				break;
			case IDLE:
				stop();
				break;
			default:
				stop();
				break;
		}
	}

	private SystemState handleStateTransitions() {
		switch (wantedState) {
			case TELEOP_DRIVE: 
				return SystemState.TELEOP_DRIVE;
			case AUTO: 
				return SystemState.AUTO;
			case IDLE:
				return SystemState.IDLE;
			default:
				return SystemState.IDLE;
		}
	}

	public void setWantedState(WantedState wantedState) {
		if (this.wantedState == wantedState) {
			return;
		}
		this.wantedState = wantedState;
	}

	public Command driveCommand(ChassisSpeeds speeds) {
		return new InstantCommand(() -> {
			setTargetChassisSpeeds(speeds);
		});
	}
	
	public Command stopCommand() {
		return new InstantCommand(() -> {
			stop();
		});
	}

	public void stop() {
		for (var module : modules) {
			module.stop();
		}
	}

	public void setTargetChassisSpeeds() {
		// Apply deadband
		double x = MathUtil.applyDeadband(primaryController.getLeftX(), DriveConstants.deadband);
		double y = MathUtil.applyDeadband(primaryController.getLeftY(), DriveConstants.deadband);
		double omega = MathUtil.applyDeadband(primaryController.getRightX(), DriveConstants.deadband);

		// Square value for more precise control
		x = Math.copySign(x * x, x);
		y = Math.copySign(y * y, y);
		omega = Math.copySign(omega * omega, omega);

		// Convert to field relative speeds
		ChassisSpeeds speeds =
			new ChassisSpeeds(
				x * DriveConstants.maxSpeedMetersPerSec,
				y * DriveConstants.maxSpeedMetersPerSec,
				omega * DriveConstants.maxAngularSpeedRadPerSec);
		boolean isFlipped =
			DriverStation.getAlliance().isPresent()
				&& DriverStation.getAlliance().get() == Alliance.Red;

		// set the bot to run at the chassis speeds
		setTargetChassisSpeeds(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				speeds, isFlipped ? getPose().getRotation().plus(new Rotation2d(Math.PI)) : getPose().getRotation()));
	}

	public void setTargetChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {
		this.targetChassisSpeeds = targetChassisSpeeds;
	}

	public SwerveModuleState[] getModuleState()
	{
		SwerveModuleState[] states = new SwerveModuleState[4];
		for(int i = 0; i < 4; i++)
		{
			states[i] = modules[i].getState();
		}
		return states;
	}
	
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}
}
