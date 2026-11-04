package frc.robot.Subsystems.Swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.DriveConstants;
import frc.robot.Constants.RobotConstants.DriveConstants.ModuleOrientation;
import frc.robot.Subsystems.Swerve.Module.ModuleIO;
import frc.robot.Subsystems.Swerve.Module.SwerveModule;
// import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
// import com.pathplanner.lib.util.swerve.SwerveSetpoint;

public class Swerve extends SubsystemBase {
	static final Lock odometryLock = new ReentrantLock();

	private final SwerveModule[] modules = new SwerveModule[4];

	private SwerveModulePosition[] lastModulePositions
		= new SwerveModulePosition[] {
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition()
		};

	private CommandXboxController primaryController;

	private SwerveDriveKinematics kinematics 
		= new SwerveDriveKinematics(DriveConstants.MODULE_OFFSETS);

	// So this needs a gyro, but we don't have a gyro. Too lazy to add one rn
	// private SwerveDrivePoseEstimator poseEstimator
	// 	= new SwerveDrivePoseEstimator(kinematics, gyroAngle, lastModulePositions, new Pose2d());

	// private final SwerveSetpointGenerator setPointGenerator;
	// private SwerveSetpoint previousSetpoint; maybe later

	private Pose2d currentPose = new Pose2d();
	private double lastTime;

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
		ModuleIO flModuleIO,
      	ModuleIO frModuleIO,
      	ModuleIO blModuleIO,
      	ModuleIO brModuleIO,
		CommandXboxController primaryController
	) {
		modules[0] = new SwerveModule(flModuleIO, ModuleOrientation.FRONT_LEFT);
    	modules[1] = new SwerveModule(frModuleIO, ModuleOrientation.FRONT_RIGHT);
    	modules[2] = new SwerveModule(blModuleIO, ModuleOrientation.BACK_LEFT);
   	 	modules[3] = new SwerveModule(brModuleIO, ModuleOrientation.BACK_RIGHT);
		this.primaryController = primaryController;
	}
	
	@Override
	public void periodic()
	{
		odometryLock.lock();
		for(var module : modules)
		{
			module.periodic();
		}
		odometryLock.unlock();

		if (DriverStation.isDisabled()) 
		{
			for (var module : modules) 
			{
				module.stop();
			}
		}

		updatePoseEstimate();
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
				SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);
				for (int i = 0; i < 4; i++) {
					modules[i].runSwerveState(moduleStates[i]);
				}
				break;
			case IDLE:
				for (var module : modules) 
				{
					module.stop();
				}
				break;
			default:
				for (var module : modules) 
				{
					module.stop();
				}
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

	public void setTargetChassisSpeeds(double xInput, double yInput, double omegaInput) {
		// Apply deadband
		double x = MathUtil.applyDeadband(xInput, DriveConstants.deadband);
		double y = MathUtil.applyDeadband(yInput, DriveConstants.deadband);
		double omega = MathUtil.applyDeadband(omegaInput, DriveConstants.deadband);

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
				speeds, isFlipped ? currentPose.getRotation().plus(new Rotation2d(Math.PI)) : currentPose.getRotation()));
	}

	public void setTargetChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {
		this.targetChassisSpeeds = targetChassisSpeeds;
	}

	// Here's what we'll do instead!!!
	// This is so cursed, like never do this again
	// If this doesn't work, just don't do field relative speeds. I really don't wanna do odom
	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d updatePoseEstimate() {
		SwerveModulePosition[] lastestModulePositions
			= new SwerveModulePosition[4];
		for(int i = 0; i < 4; i++) {
			lastestModulePositions[i] = modules[i].getLatestPosition();
		}
		currentPose = currentPose.exp(kinematics.toTwist2d(lastModulePositions, lastestModulePositions));
		lastModulePositions = lastestModulePositions;
		return currentPose;
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
	public SwerveModulePosition[] getModulePosition()
	{
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for(int i = 0; i < 4; i++)
		{
			positions[i] = new SwerveModulePosition(modules[i].getPosition_Meters(), modules[i].getAngle());
		}
		return positions;
	}
}
