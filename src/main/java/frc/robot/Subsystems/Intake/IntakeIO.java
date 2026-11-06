package frc.robot.Subsystems.Intake;

public interface IntakeIO{
    @Autolog
    public class IntakeIOInputs{
        public double spinnerIntakeVoltage = 0.0;
        public double spinnerIntakeSupplyCurrent = 0.0;
        public double spinnerIntakeStatorCurrent = 0.0;
        public double spinnerIntakeTemperature = 0.0;
        public boolean isConnected=false;
       
    }
    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setExtensionsMotorPositionRad(double rad, double velocityRotPerSec, double accelerationRotPerSec) {}

    public default void setExtensionVoltage(double voltage) {}

    public default void setSpinnerVoltage (double voltage) {}
}