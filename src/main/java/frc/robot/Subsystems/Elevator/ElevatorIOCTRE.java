package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.units.measure.Voltage;
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
        elevatorMotor = new TalonFX(RobotConstants.ElevatorConstants.elevatorCanID(), RobotConstants.ElevatorConstants.getBus());
        
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
        return elevatorMotor.getPosition();
    }
    @Override
    public double getVelocity() {
        return elevatorMotor.getVelocity();
    }
    @Override
    public void resetEncoder() {
        elevatorMotor.setPosition(0.0);

    }
    @Override        
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll()
        BaseStatusSignal.refreshAll()
        inputs.elevatorCTREVoltage = elevationAppliedVolts.getValueAsDouble();
        inputs.elevatorCTREPosition = elevationSupplyCurrentAmps.getValueAsDouble();
        inputs.elevationStatorCurrent = elevationStatorCurrentAmps.getValueAsDouble();
        inputs.elevationTemperature = elevationMotorTemp.getValueAsDouble();



        }
    }


    


}
