package frc.robot.Subsystems.Intake;

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
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Constant.Constants;
import frc.robot.Util.Phoenix6Util;



public class IntakeIOCTRE implements IntakeIO{

 private TalonFX spinnerIntakeMotor;

 private TalonFXConfiguration spinnerConfig;


  private final StatusSignal<Voltage> spinnerAppliedVolts;
  private final StatusSignal<Current> spinnerSupplyCurrentAmps;
  private final StatusSignal<Current> spinnerStatorCurrentAmps;
  private final StatusSignal<AngularVelocity> spinnerVelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> spinnerAccelerationRotationsPerSecSquared;
  private final StatusSignal<Temperature> spinnerMotorTemp;

 public IntakeIOCTRE(BruinRobotConfig robotConfig) {
    spinnerIntakeMotor =
        new TalonFX(
            robotConfig.INTAKE_ROLLER.getDeviceNumber(), robotConfig.INTAKE_ROLLER.getBus());


   spinnerConfig = new TalonFXConfiguration();
    spinnerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    spinnerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    spinnerConfig.CurrentLimits.SupplyCurrentLimit = 100.0;
    spinnerConfig.CurrentLimits.StatorCurrentLimit = 100.0;

    spinnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    spinnerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      Phoenix6Util.applyAndCheckConfiguration(spinnerIntakeMotor, spinnerConfig, 5);

      
    spinnerAppliedVolts = spinnerIntakeMotor.getMotorVoltage();
    spinnerSupplyCurrentAmps = spinnerIntakeMotor.getSupplyCurrent();
    spinnerStatorCurrentAmps = spinnerIntakeMotor.getStatorCurrent();
    spinnerVelocityRotationsPerSec = spinnerIntakeMotor.getVelocity();
    spinnerAccelerationRotationsPerSecSquared = spinnerIntakeMotor.getAcceleration();
    spinnerMotorTemp = spinnerIntakeMotor.getDeviceTemp();






}
@Override
  public void updateInputs(IntakeIOInputs inputs) {
 BaseStatusSignal.refreshAll(
        spinnerAppliedVolts,
        spinnerSupplyCurrentAmps,
        spinnerStatorCurrentAmps,
        spinnerVelocityRotationsPerSec,
        spinnerAccelerationRotationsPerSecSquared,
        spinnerMotorTemp);

    inputs.spinnerIntakeVoltage = spinnerAppliedVolts.getValueAsDouble();
    inputs.spinnerIntakeSupplyCurrent = spinnerSupplyCurrentAmps.getValueAsDouble();
    inputs.spinnerIntakeStatorCurrent = spinnerStatorCurrentAmps.getValueAsDouble();
    inputs.spinnerIntakeVelocityRadPerSec =
        Units.rotationsToRadians(
            spinnerVelocityRotationsPerSec.getValueAsDouble()); // Update with constant
    inputs.spinnerIntakeAccelRadPerSecSquared =
        Units.rotationsToRadians(spinnerAccelerationRotationsPerSecSquared.getValueAsDouble());
    inputs.spinnerIntakeTemperature = spinnerMotorTemp.getValueAsDouble();


  }

@Override
  public void setSpinnerVoltage(double voltage) {
    spinnerIntakeMotor.setVoltage(voltage);
  }
}