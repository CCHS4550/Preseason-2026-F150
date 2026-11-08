package frc.robot.Constants;
import com.ctre.phoenix6.CANBus;
import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class RobotConstants {
    public static class GrabberConstants {
		public static final int CanID = 0;
		public static final CANBusbus = CANBus.roboRIO();
		public static final InvertedValue direction = InvertedValue.CounterClockwise_Positive;

		public static final double intakeVoltage = 1.0;
	}
}

