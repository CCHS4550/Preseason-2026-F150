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
    public static final CANBus CARNIVORE_BUS = new CANBus("spinnerIntakeMotor");

    public static final int intakeMotorID = 0;
}

