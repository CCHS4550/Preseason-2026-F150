package frc.robot.Subsystems.Swerve.Gyro;

import java.util.Queue;

import com.ctre.phoenix6.StatusSignal;
import com.google.flatbuffers.Constants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.RobotConstants.DriveConstants;

public class GyroIOPigeon implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(DriveConstants.pigeonCanId);

    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

    public GyroIOPigeon () {
        
    }
}
