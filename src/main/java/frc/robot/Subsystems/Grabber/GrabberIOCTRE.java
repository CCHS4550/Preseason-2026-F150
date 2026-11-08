package frc.robot.Subsystems.Grabber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Util.Phoenix6Util;


public class GrabberIOCTRE implements GrabberIO{

 	private TalonFX grabberMotor;
 
 	private TalonFXConfiguration grabberConfig;

  	private final StatusSignal<Voltage> appliedVolts;
  	private final StatusSignal<Current> supplyCurrentAmps;
  	private final StatusSignal<Current> statorCurrentAmps;
  	private final StatusSignal<AngularVelocity> velocityRotationsPerSec;
  	private final StatusSignal<AngularAcceleration> accelerationRotationsPerSecSquared;

 	public GrabberIOCTRE() {
    	grabberMotor = new TalonFX(
			GrabberConstants.CanID,
			new CANBus(GrabberConstants.bus)
		);

   		grabberConfig = new TalonFXConfiguration();
    	grabberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    	grabberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    	grabberConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
    	grabberConfig.CurrentLimits.StatorCurrentLimit = 70.0;

    	grabberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    	grabberConfig.MotorOutput.Inverted = GrabberConstants.direction;

      	if(grabberMotor.getConfigurator().apply(grabberConfig) != StatusCode.OK) {
            throw new RuntimeException("Failed to apply Grabber configs: " + statusCode);
        }

		appliedVoltage = grabberMotor.getMotorVoltage();
		supplyCurrentAmps = grabberMotor.getSupplyCurrent();
		statorCurrentAmps = grabberMotor.getStatorCurrent();
		velocityRotationsPerSec = grabberMotor.getVelocity();
		accelerationRotationsPerSecSquared = grabberMotor.getAcceleration();
	}

	@Override
	public void updateInputs(GrabberIOInputs inputs) {
	BaseStatusSignal.refreshAll(
			appliedVolts,
			supplyCurrentAmps,
			statorCurrentAmps,
			velocityRotationsPerSec,
			accelerationRotationsPerSecSquared);

		inputs.appliedVoltage = appliedVolts.getValueAsDouble();
		inputs.supplyCurrent = supplyCurrentAmps.getValueAsDouble();
		inputs.statorCurrent = statorCurrentAmps.getValueAsDouble();
		inputs.velocityRotationsPerSec = velocityRotationsPerSec.getValueAsDouble(); // Update with constant
		inputs.accelRotationsPerSecSquared = accelerationRotationsPerSecSquared.getValueAsDouble();
	}

	@Override
	public void setVoltage(double voltage) {
		grabberMotor.setVoltage(voltage);
	}
}