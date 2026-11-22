package frc.robot.Subsystems.Swerve.Module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
        public double   turnEncoderVoltage_Volts = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

        public SwerveModulePosition currentState = new SwerveModulePosition();
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVelocity(double velocity_RadPerSec) {}

    public default double setTurnAngle(Rotation2d rotation) {return 0.0;}

    public default void setDriveVoltage(double voltage) {}

    public default void setTurnVoltage(double voltage) {}

    public default double getRawRadians() {return 0.0;}
}