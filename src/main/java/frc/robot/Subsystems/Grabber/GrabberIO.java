package frc.robot.Subsystems.Grabber;

public interface GrabberIO{
    @Autolog
    public class GrabberIOInputs{
        public double appliedVoltage = 0.0;
        public double supplyCurrent = 0.0;
        public double statorCurrent = 0.0;
        public double velocityRotationsPerSec = 0.0;
        public double accelerationRotationsPerSecSquared = 0.0;
    }
    public default void updateInputs(GrabberIOInputs inputs) {}

    public default void setVoltage(double voltage) {}
}