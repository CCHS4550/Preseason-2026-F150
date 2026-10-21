// make sure to set these later

import edu.wpi.first.math.geometry.Translation2d;

public final class RobotConstants {
	//not all constants from module io spark are in here
	public static final class DriveConstants {
		public static enum ModuleOrientation {
			FRONT_LEFT,
			FRONT_RIGHT,
			BACK_LEFT,
			BACK_RIGHT
		}

		public static final double WHEEL_RADIUS_METERS = 1.0;
		public static final Translation2d[] MODULE_OFFSETS = {};

		public static final int frontLeftTurnCanId = 69;
		public static final int frontRightTurnCanId = 68;
		public static final int backRightTurnCanId = 67;
		public static final int backLeftTurnCanId = 66;


		public static final int frontLeftDriveCanId = 12;
		public static final int frontRightTurnCanId = 13;
		public static final int backRightTurnCanId = 257;
		public static final int backLeftTurnCanId = 2525;
	}
}