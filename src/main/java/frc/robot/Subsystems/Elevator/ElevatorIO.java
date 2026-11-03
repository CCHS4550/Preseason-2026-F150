package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIoInputs {
        public boolean elevatorCTREConnected = false;
        public double elevatorCTREPosition = 0.0;
        public double elevatorCTREVelocity = 0.0;
        public double elevatorCTREVoltage = 0.0;
    }
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
    public default void updateInputs(ElevatorIoInputs inputs) {}
}
