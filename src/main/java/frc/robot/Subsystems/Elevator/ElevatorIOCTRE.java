package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOCTRE implements ElevatorIO {
    
    private TalonFX elevatorMotor;
    private TalonFXConfiguration elevatorConfig;
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

    public ElevatorIOCTRE(){
        motionMagicVoltage = new MotionMagicVoltage(0.0);
        elevatorMotor = new TalonFX(Constants.ElevatorConstants.elevatorCanID(), Constants.ElevatorConstants.getBus());
        elevatorEncoder = new CANcoder();
        encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.withMagnetOffset();

        @Override 
        public void stopElevator() {
            elevatorMotor.stopMotor();



        }
        @Override
        public void setSpeed(double speed) {
            elevatorMotor.set(speed);

        }
        @Override
        public void setVoltage(double volts) {
            elevatorMotor.setVoltage(volts);
        }
        @Override
        public void setPosition(double position) {
            elevatorMotor.setPosition(position);
        }
        @Override
        public double getPosition() {
            return 0.0;
        }
        @Override
        public double getVelocity() {
            return 0.0;
        }
        @Override
        public void resetEncoder() {

        }
        @Override        
        public void updateInputs(ElevatorIoInputs inputs) {
            

        }
    }


    


}
