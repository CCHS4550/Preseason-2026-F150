package frc.robot.Constants;

import com.ctre.phoenix6.CANBus;

public final class RobotConstants {
    public static class ElevatorConstants{
        public static final int elevatorCanID = 999;
		public static final CANBus bus = CANBus.roboRIO();

        public static final InvertedValue elevatorDirection = InvertedValue.CounterClockwise_Positive;

		public static final double kP = 1.0;
		public static final double kI = 0.0;
		public static final double kD = 0.0;

		public static final double kS = 0.5;
		public static final double kV = 0.0;
		public static final double kG = 0.5;

		public static final double conversionFactor = 1.0; // rotations / meters
        public static final double levelTwoHeightMeters = 30; //placeholder change later
        public static final double levelOneHeightMeters = 15; //placeholder change later
        public static final double groundHeightMeters = 0; //placeholder change later
        public static final double driverStationHeightMeters = 0; //placeholder change later
    }
}