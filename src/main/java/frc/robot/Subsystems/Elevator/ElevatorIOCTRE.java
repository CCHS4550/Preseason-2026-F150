package frc.robot.Subsystems.Elevator;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOCTRE implements ElevatorIO {

    private final TalonFX elevatorMotor;
    private final TalonFXConfiguration elevatorConfig;

    private final MotionMagicVoltage elevatorController = new MotionMagicVoltage(0.0);

    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> supplyCurrentAmps;
    private final StatusSignal<Current> statorCurrentAmps;
    private final StatusSignal<Angle> positionRotations;
    private final StatusSignal<AngularVelocity> velocityRotationsPerSec;
    private final StatusSignal<AngularAcceleration> accelerationRotationsPerSecSquared;

    public ElevatorIOCTRE() {
        elevatorMotor = new TalonFX(
                ElevatorConstants.elevatorCanID,
            new CANBus(ElevatorConstants.bus));

        elevatorConfig = new TalonFXConfiguration();
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        elevatorConfig.CurrentLimits.StatorCurrentLimit = 90.0;

        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfig.MotorOutput.Inverted = ElevatorConstants.elevatorDirection;

        elevatorConfig.Slot0.kP = ElevatorConstants.kP;
        elevatorConfig.Slot0.kI = ElevatorConstants.kI;
        elevatorConfig.Slot0.kD = ElevatorConstants.kD;
        elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        elevatorConfig.Slot0.kS = ElevatorConstants.kS;
        elevatorConfig.Slot0.kV = ElevatorConstants.kV;
        elevatorConfig.Slot0.kG = ElevatorConstants.kG;
        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        elevatorConfig.MotionMagic.MotionMagicAcceleration = 50;

        if(elevatorMotor.getConfigurator().apply(elevatorConfig) != StatusCode.OK) {
            throw new RuntimeException("Failed to apply Elevator configs: " + statusCode);
        }

        elevatorMotor.setPosition(ElevatorConstants.groundHeightMeters * ElevatorConstants.conversionFactor);
        positionRotations = elevatorMotor.getPosition();
        appliedVoltage = elevatorMotor.getMotorVoltage();
        supplyCurrentAmps = elevatorMotor.getSupplyCurrent();
        statorCurrentAmps = elevatorMotor.getStatorCurrent();
        velocityRotationsPerSec = elevatorMotor.getVelocity();
        accelerationRotationsPerSecSquared = elevatorMotor.getAcceleration();
    }

    @Override
    public void stop() {
        elevatorMotor.stopMotor();
    }

    @Override
    public void setSpeed(double speed) {
        elevatorMotor.set(speed);
    }

    @Override
    public void setVoltage(double volts) {
        elevatorMotor.setVoltage(volts);
    }

    @Override
    public void setPositionMeters(double positionMeters, double veloRotPerSec, double accelRotPerSec) {
        elevatorMotor.setControl(motionMagicVoltage
            .withPosition(positionMeters * ElevatorConstants.conversionFactor)
            .withVelocity(veloRotPerSec)
            .withAcceleration(accelRotPerSec)
            );
    }

    @Override
    public double getPositionMeters() {
        return elevatorMotor.getPosition().getValueAsDouble() / ElevatorConstants.conversionFactor;
    }

    @Override
    public double getVelocity() {
        return elevatorMotor.getVelocity().getValueAsDouble() / ElevatorConstants.conversionFactor;
    }

    @Override
    public void resetEncoder() {
        elevatorMotor.setPosition(ElevatorConstants.groundHeightMeters * ElevatorConstants.conversionFactor);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            appliedVoltage,
            supplyCurrentAmps, 
            statorCurrentAmps, 
            positionRotations,
            velocityRotationsPerSec,
            accelerationRotationsPerSecSquared);
        
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
        inputs.positionMeters = positionRotations.getValueAsDouble() / ElevatorConstants.conversionFactor;
        inputs.velocityMetersPerSec = velocityRotationsPerSec.getValueAsDouble() / ElevatorConstants.conversionFactor;
        inputs.accelerationMetersPerSecSquared = accelerationRotationsPerSecSquared.getValueAsDouble() / ElevatorConstants.conversionFactor;   
    } 
}