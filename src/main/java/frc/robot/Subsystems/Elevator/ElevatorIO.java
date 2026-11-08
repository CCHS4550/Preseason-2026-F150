package frc.robot.Subsystems.Elevator;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double appliedVoltage = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double accelerationMetersPerSecSquared = 0.0;
    }

    public default void stop() {
    }

    public default void setSpeed(double speed) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setPositionMeters(double positionMeters, double veloRotPerSec, double accelRotPerSec) {
    }

    public default double getPositionMeters() {
        return 0.0;
    } 

    public default double getVelocity() {
        return 0.0;
    }

    public default void resetEncoder() {
    }
    public default void updateInputs(ElevatorIOInputs inputs) {
    }
}
