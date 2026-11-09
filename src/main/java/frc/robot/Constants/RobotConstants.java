package frc.robot.Constants;
import com.ctre.phoenix6.CANBus;

import com.ctre.phoenix6.signals.InvertedValue;

public class RobotConstants {
    public static class GrabberConstants {
		public static final int CanID = 0;
		public static final CANBus bus = CANBus.roboRIO();
		public static final InvertedValue direction = InvertedValue.CounterClockwise_Positive;

		public static final double intakeVoltage = 1.0;
	}
}

