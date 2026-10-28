package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSpark implements ElevatorIO {
    
    SparkMax elevatorSpark = new SparkMax(Constants.ElevatorConstants.elevatorCanID, MotorType.kBrushless);
    
    @Override
    public default void stopElevator() {
        elevatorSpark
    }

    @Override
    public default void setSpeed(double speed) {
    }

    @Override
    public default void setVoltage(double volts) {
    }

    @Override
    public default void setPosition(double position) {
    }

    @Override
    public default double getPosition() {
        return 0.0;
    }

    @Override
    public default double getVelocity() {
        return 0.0;
    }

    @Override
    public default void resetEncoder() {
    }



}
