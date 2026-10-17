package frc.robot.Subsystems.Swerve.Module;
import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean  driveConnected = false;
        public double   drivePosition_Rad = 0.0;
        public double   driveVelocity_RadPerSec = 0.0;
        public double   driveAppliedVoltage_Volts = 0.0;
        public double   driveCurrent_Amps = 0.0;

        public boolean  turnConnected = false;
        public Rotation2d turnPosition = new Rotation2d();
        public double   turnVelocity_RadPerSec = 0.0;
        public double   turnAppliedVoltage_Volts = 0.0;
        public double   turnCurrent_Amps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVelocity(double velocity_RadPerSec) {}

    public default void setTurnAngle(Rotation2d rotation) {}

    public default void setDriveOpenLoop(double voltage) {}

    public default void setTurnOpenLoop(double voltage) {}
}