package frc.robot.Subsystems.Swerve.Module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.RobotConstants.DriveConstants;
import frc.robot.Constants.RobotConstants.DriveConstants.ModuleOrientation;

// add logging if you feel like it

public class SwerveModule {
	private final ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private final DriveConstants.ModuleOrientation moduleOrientation;

	private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

	public SwerveModule(ModuleIO io, DriveConstants.ModuleOrientation moduleOrientation) {
		this.io = io;
		this.moduleOrientation = moduleOrientation;
		reset();
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

		Logger.processInputs(switch(moduleOrientation) {
			case FRONT_LEFT -> "Swerve/FL_Module";
			case FRONT_RIGHT -> "Swerve/FR_Module";
			case BACK_LEFT -> "Swerve/BL_Module";
			case BACK_RIGHT -> "Swerve/BR_Module";
		}, inputs);
	}

	public void runSwerveState(SwerveModuleState state) {
		state.optimize(getAngle());
		state.cosineScale(getAngle());
		System.out.printf("%s angle: %.2f speed: %.2f\n", moduleOrientation, state.angle.getDegrees(), state.speedMetersPerSecond);

		io.setDriveVelocity(state.speedMetersPerSecond / DriveConstants.WHEEL_RADIUS_METERS);
		io.setTurnAngle(state.angle);
		
	}

	public void stop() {
		io.setDriveVoltage(0.0);
		io.setTurnVoltage(0.0);
	}

	public void runOpenLoop(double voltage) {
		io.setTurnVoltage(voltage);
		io.setDriveVoltage(voltage);
	}

	public void reset() {
		io.setTurnAngle(new Rotation2d(0.0));
		io.setDriveVelocity(0.0);
	}

	public Rotation2d getAngle() {
		Rotation2d angle = inputs.turnPosition.times(
		switch(moduleOrientation) {
			case BACK_RIGHT -> DriveConstants.backRightTurnInverted ? 1.0 : -1.0;
			case FRONT_LEFT -> DriveConstants.frontLeftDriveInverted ? 1.0 : -1.0;
			case BACK_LEFT -> DriveConstants.backLeftTurnInverted ? 1.0 : -1.0;
			case FRONT_RIGHT -> DriveConstants.frontRightDriveInverted ? 1.0 : -1.0;
		});
		if(angle.getRadians() > 2.0 * Math.PI) {
			angle.minus(Rotation2d.fromRadians(2.0 * Math.PI));
		}
		if(angle.getRadians() < 0.0) {
			angle.plus(Rotation2d.fromRadians(2.0 * Math.PI));
		}
		return angle;
	}

	public double getPosition_Meters() {
		return inputs.drivePosition_Rad * DriveConstants.WHEEL_RADIUS_METERS;
	}

	public double getVelocity_MetersPerSec() {
		return getPosition_Meters() 
				* DriveConstants.WHEEL_RADIUS_METERS;
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getPosition_Meters(), getAngle());
	}

	public double getRawRadians() {
		return io.getRawRadians();
	}

	public SwerveModulePosition getLatestPosition() {
		return new SwerveModulePosition(getPosition_Meters(), getAngle());
	}

	public SwerveModulePosition[] getOdometryPositions() {
		return odometryPositions;
	}

	public double[] getOdometryTimestamps() {
		return inputs.odometryTimestamps;
	}

}