package frc.robot.Subsystems.Swerve.Module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.RobotConstants.DriveConstants;
import frc.robot.Constants.TestingOnly;
import frc.robot.Subsystems.Swerve.Module.ModuleIO.ModuleIOInputs;

// add logging if you feel like it

public class SwerveModule {
	private final ModuleIO io;
	private final ModuleIOInputs inputs = new ModuleIOInputs();
	private final DriveConstants.ModuleOrientation moduleOrientation;

	private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

	public SwerveModule(ModuleIO io, DriveConstants.ModuleOrientation moduleOrientation) {
		this.io = io;
		this.moduleOrientation = moduleOrientation;
	}

	public void periodic() {
		io.updateInputs(inputs);

		int sampleCount = inputs.odometryTimestamps.length;
		odometryPositions = new SwerveModulePosition[sampleCount];

		for(int i = 0; i < sampleCount; i++) {
			double position_Meters = inputs.odometryDrivePositionsRad[i] 
				* DriveConstants.WHEEL_RADIUS_METERS;
			Rotation2d turnAngle = inputs.odometryTurnPositions[i];
			odometryPositions[i] = new SwerveModulePosition(position_Meters, turnAngle);
		}
	}

	public void runSwerveState(SwerveModuleState state) {
		state.optimize(getAngle());
		state.cosineScale(getAngle());

		io.setDriveVelocity(state.speedMetersPerSecond / DriveConstants.WHEEL_RADIUS_METERS);
		io.setTurnAngle(state.angle);
	}

	public void stop() {
		io.setDriveOpenLoop(0.0);
		io.setTurnOpenLoop(0.0);
	}

	@TestingOnly
	public void runOpenLoop(double voltage) {
		io.setTurnOpenLoop(voltage);
		io.setDriveOpenLoop(voltage);
	}

	@TestingOnly
	public void reset() {
		io.setTurnAngle(new Rotation2d(0.0));
		io.setDriveVelocity(0.0);
	}

	public Rotation2d getAngle() {
		return inputs.turnPosition;
	}

	public double getPosition_Meters() {
		return inputs.drivePosition_Rad;
	}

	public double getVelocity_MetersPerSec() {
		return getPosition_Meters() 
				* DriveConstants.WHEEL_RADIUS_METERS;
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getPosition_Meters(), getAngle());
	}

	public SwerveModulePosition[] getOdometryPositions() {
		return odometryPositions;
	}

	public double[] getOdometryTimestamps() {
		return inputs.odometryTimestamps;
	}

}