package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    public default void stopElevator() {
    }

    public default void setSpeed(double speed) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setPosition(double position) {
    }

    public default double getPosition() {
        return 0.0;
    }

    public default double getVelocity() {
        return 0.0;
    }

    public default void resetEncoder() {
    }

}
