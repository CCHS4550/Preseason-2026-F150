package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Voltage;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSpark implements ElevatorIO {
    private final SparkBase elevatorSpark;
    private final RelativeEncoder elevatorEncoder;
    public ElevatorIOSpark(){

        SparkMax elevatorSpark = new SparkMax(Constants.ElevatorConstants.elevatorCanID, MotorType.kBrushless);
        elevatorEncoder = elevatorSpark.getEncoder();
    
        @Override
        public void stopElevator() {
            elevatorSpark.setVoltage(0.0);
        }

        @Override
        public void setSpeed(double speed) {
            elevatorEncoder.setVelocity(speed);
     }

        @Override
        public void setVoltage(double volts) {
            elevatorSpark.setVoltage(volts);
        }

        @Override
        public void setPosition(double position) {
            elevatorEncoder.setPosition(position);
        }

        @Override
        public double getPosition() {
            return elevatorEncoder.getPosition();
        }

        @Override
        public double getVelocity() {
            return elevatorEncoder.getVelocity();
        }

        @Override
        public void resetEncoder() {
            elevatorEncoder.setPosition(0);

        }
    }


}
