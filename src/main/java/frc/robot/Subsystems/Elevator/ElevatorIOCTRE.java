package frc.robot.Subsystems.Elevator;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOCTRE implements ElevatorIO {

    private final TalonFX elevatorMotor;
    private final MotionMagicVoltage motionMagicVoltage;
    private final StatusSignal<Angle> elevPosition;
    private final StatusSignal<Voltage> elevAppliedVoltage;
    private final StatusSignal<Current> elevCurrent;
    private final StatusSignal<AngularVelocity> elevVelocity;

    public ElevatorIOCTRE() {
        motionMagicVoltage = new MotionMagicVoltage(0.0);
        elevatorMotor = new TalonFX(
                RobotConstants.ElevatorConstants.elevatorCanID(),
            new CANBus(RobotConstants.ElevatorConstants.getBus()));
        
        elevPosition = elevatorMotor.getPosition();
        elevAppliedVoltage = elevatorMotor.getMotorVoltage();
        elevCurrent = elevatorMotor.getSupplyCurrent();
        elevVelocity = elevatorMotor.getVelocity();
    }

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
        elevatorMotor.setControl(motionMagicVoltage.withPosition(position));
    }

    @Override
    public double getPosition() {
        return elevatorMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return elevatorMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void resetEncoder() {
        elevatorMotor.setPosition(0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
            BaseStatusSignal.refreshAll(elevAppliedVoltage, elevCurrent, elevPosition, elevVelocity);
        
            inputs.elevatorCTREPosition = elevPosition.getValueAsDouble();
            inputs.elevatorCTREVelocity = elevVelocity.getValueAsDouble();
            inputs.elevatorCTREVoltage = elevAppliedVoltage.getValueAsDouble();
            inputs.elevatorCTRECurrent = elevCurrent.getValueAsDouble();
            
            
            
}

final class RobotConstants {
    public static final class ElevatorConstants {
        public static int elevatorCanID() {
            return 0;
        }

        public static String getBus() {
            return "rio";
        }
    }

}
}