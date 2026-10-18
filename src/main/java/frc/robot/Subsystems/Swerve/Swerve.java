package frc.robot.Subsystems.Swerve;

import frc.robot.Constants.DriveConstants;

public class Swerve extends SubsystemBase {
	private final Module[] modules = new Module[4];

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

		

}