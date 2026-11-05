package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

// make sure to set these later

public final class RobotConstants {
	//not all constants from module io spark are in here
	public static final class DriveConstants {
		public static enum ModuleOrientation {
			FRONT_LEFT,
			FRONT_RIGHT,
			BACK_LEFT,
			BACK_RIGHT
		}

		public static final double deadband = 0.2;

		// Taken with measuring tape, distance between centers of wheels
		public static final double trackWidth = Units.inchesToMeters(19.375);
		public static final double wheelBase = Units.inchesToMeters(22.25);
		public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
		public static final double odometryFrequency = 80.0; // Hz

		public static final double maxSpeedMetersPerSec = 4.8;
		public static final double maxAngularSpeedRadPerSec = maxSpeedMetersPerSec / driveBaseRadius;

		public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);;
		public static final Translation2d[] MODULE_OFFSETS = new Translation2d[] {
         	new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          	new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          	new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          	new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

		public static final Rotation2d frontLeftOffset = Rotation2d.fromRadians(0.0);
    	public static final Rotation2d frontRightOffset = Rotation2d.fromRadians(0.0);
    	public static final Rotation2d backRightOffset = Rotation2d.fromRadians(0.0);
    	public static final Rotation2d backLeftOffset = Rotation2d.fromRadians(0.0);

		public static final int frontLeftTurnCanId = 69;
		public static final int frontRightTurnCanId = 68;
		public static final int backRightTurnCanId = 67;
		public static final int backLeftTurnCanId = 66;


		public static final int frontLeftDriveCanId = 12;
		public static final int frontRightDriveCanId = 13;
		public static final int backRightDriveCanId = 257;
		public static final int backLeftDriveCanId = 2525;

		public static final boolean frontLeftTurnInverted = true;
		public static final boolean frontRightTurnInverted = true;
		public static final boolean backLeftTurnInverted = true;
		public static final boolean backRightTurnInverted = true;

		public static final boolean frontLeftDriveInverted = true;
		public static final boolean frontRightDriveInverted = true;
		public static final boolean backLeftDriveInverted = false;
		public static final boolean backRightDriveInverted = false;

		 public static final int driveMotorCurrentLimit = 60;
		public static final double wheelRadiusMeters = Units.inchesToMeters(2);
		public static final double driveMotorReduction = 6.12;
		public static final double driveEncoderPositionFactor =
			2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
		public static final double driveEncoderVelocityFactor =
			(2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

		public static final int turnMotorCurrentLimit = 60;
		public static final double turnMotorReduction = 12.8;
		public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

		// Turn encoder configuration
		// TODO: URGENT, apply the gear reduction
		public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
		public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

		public static final int frontRightTurnEncoder = 0;
    	public static final int frontLeftTurnEncoder = 2;
    	public static final int backRightTurnEncoder = 1;
    	public static final int backLeftTurnEncoder = 3;

		// all borrowed from cherry blossom
		// Drive PID configuration
		public static final double driveKp = 0.01;
		public static final double driveKd = 0.0;

		public static final double driveKs = 0.16681;
		public static final double driveKv = 2.609 - 2;
		public static final double driveKa = 0.51582;

   		 // Turn PID configuration
		public static final double turnKp = 3.14;
		public static final double turnKi = 0.0;
		public static final double turnKd = 0.0;
		public static final double turnPIDMinInput = 0; // Radians
		public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

		public static Object pigeonCanId;
	}
}