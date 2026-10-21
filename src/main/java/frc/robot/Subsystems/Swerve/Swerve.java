package frc.robot.Subsystems.Swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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


	private SwerveDriveKinematics kinematics 
		= new SwerveDriveKinematics(DriveConstants.MODULE_OFFSETS);

	// So this needs a gyro, but we don't have a gyro. Too lazy to add one rn
	//private SwerveDrivePoseEstimator poseEstimator
	//	= new SwerveDrivePoseEstimator(kinematics, gyroAngle, lastModulePositions, new Pose2d());

	// private final SwerveSetpointGenerator setPointGenerator;
	// private SwerveSetpoint previousSetpoint; maybe later


	private double xJoystickInput = 0.0;
  	private double yJoystickInput = 0.0;
  	private double omegaJoystickInput = 0.0;
		
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

	private SystemState systemState = SystemState.TELEOP_DRIVE;
	private WantedState wantedState = WantedState.TELEOP_DRIVE;

	public Swerve(
		ModuleIO flModuleIO,
      	ModuleIO frModuleIO,
      	ModuleIO blModuleIO,
      	ModuleIO brModuleIO
	) {

		modules[0] = new SwerveModule(flModuleIO, ModuleOrientation.FRONT_LEFT);
    	modules[1] = new SwerveModule(frModuleIO, ModuleOrientation.FRONT_RIGHT);
    	modules[2] = new SwerveModule(blModuleIO, ModuleOrientation.BACK_LEFT);
   	 	modules[3] = new SwerveModule(brModuleIO, ModuleOrientation.BACK_RIGHT);
	}
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


}
