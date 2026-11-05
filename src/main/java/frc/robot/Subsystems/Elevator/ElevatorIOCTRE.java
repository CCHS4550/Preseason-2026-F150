package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOCTRE implements ElevatorIO {

    private final TalonFX elevatorMotor;
    private final MotionMagicVoltage motionMagicVoltage;

    public ElevatorIOCTRE() {
        motionMagicVoltage = new MotionMagicVoltage(0.0);
        elevatorMotor = new TalonFX(
                RobotConstants.ElevatorConstants.elevatorCanID(),
            new CANBus(RobotConstants.ElevatorConstants.getBus()));
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
        BaseStatusSignal.refreshAll(
                elevatorMotor.getPosition(),
                elevatorMotor.getVelocity(),
                elevatorMotor.getMotorVoltage(),
                elevatorMotor.getSupplyCurrent(),
                elevatorMotor.getStatorCurrent(),
                elevatorMotor.getDeviceTemp());
    }
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
