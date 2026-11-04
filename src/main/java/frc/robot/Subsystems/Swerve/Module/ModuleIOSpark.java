package frc.robot.Subsystems.Swerve.Module;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

//turn stuff
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.PIDController;

//USING CLOSED LOOP

import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants.RobotConstants.DriveConstants;
import frc.robot.Constants.RobotConstants.DriveConstants.ModuleOrientation;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;

// initiate the hardware and define interface methods
// call 4 times for all 4 modules

//drive is for linear speed and moving
//turn is to turn the wheel wow 
public class ModuleIOSpark implements ModuleIO {

    //0 -> front left 1 -> front right 2 -> back left, 3 -> back right
    private final SparkBase driveSpark;
    private final SparkBase turnSpark;

    //CLOSED LOOP CONTROLLER
    private final SparkClosedLoopController turnClosedLoopController;
    private final SparkClosedLoopController driveClosedLoopController;

    private final PIDController turnPID =
        new PIDController(DriveConstants.turnKp, 0.0, DriveConstants.turnKd);

    //encoders (only need relative for drive)
    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    //account for offset (true angle and encoder)
    private final Rotation2d rotationOffset;

    //odometry

    //disconnection checks
    private final Debouncer driveDebouncer = new Debouncer(0.5);
    private final Debouncer turnDebouncer = new Debouncer(0.5);

    //analog stuff for volts
    private final AnalogEncoder absoluteEncoder;
    private final AnalogInput absoluteAnalogInput;

    //CONSTRUCTOR
    public ModuleIOSpark(ModuleOrientation module) { 
        rotationOffset =
            switch (module) {
                case FRONT_LEFT -> DriveConstants.frontLeftOffset;
                case FRONT_RIGHT -> DriveConstants.frontRightOffset;
                case BACK_LEFT -> DriveConstants.backLeftOffset;
                case BACK_RIGHT -> DriveConstants.backRightOffset;
                default -> new Rotation2d();
            };


        turnSpark = new SparkMax(
            switch (module) {
                case FRONT_LEFT -> DriveConstants.frontLeftTurnCanId;
                case FRONT_RIGHT -> DriveConstants.frontRightTurnCanId;
                case BACK_LEFT -> DriveConstants.backLeftTurnCanId;
                case BACK_RIGHT -> DriveConstants.backRightTurnCanId;
                default -> 0;
            },
            MotorType.kBrushless);
        

        //just takes the encoders and converts it so we understand it
        absoluteAnalogInput = new AnalogInput(
            switch (module) {
                case FRONT_LEFT -> DriveConstants.frontLeftTurnEncoder;
                case FRONT_RIGHT -> DriveConstants.frontRightTurnEncoder;
                case BACK_LEFT -> DriveConstants.backLeftTurnEncoder;
                case BACK_RIGHT -> DriveConstants.backRightTurnEncoder;
                default -> 0;
            });
        //define
        absoluteEncoder = new AnalogEncoder(absoluteAnalogInput);


        driveSpark = new SparkMax(
            switch (module) {
                case FRONT_LEFT -> DriveConstants.frontLeftDriveCanId;
                case FRONT_RIGHT -> DriveConstants.frontRightDriveCanId;
                case BACK_LEFT -> DriveConstants.backLeftDriveCanId;
                case BACK_RIGHT -> DriveConstants.backRightDriveCanId;
                default -> 0;
            },
            MotorType.kBrushless);

        driveEncoder = driveSpark.getEncoder();
        turnEncoder = turnSpark.getAbsoluteEncoder();


        driveclosedLoopController = driveSpark.getClosedLoopController(); 
        turnclosedLoopController = turnSpark.getClosedLoopController(); 
        //ADD PID STUFF CONFIG
        turnPID.enableContinuousInput(0, Math.PI);
        //configs
        //turn
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        //some are inverted we just set the right ones with FRONT_LEFT-3 to set the right motor
        switch (module) {
            case FRONT_LEFT : turnConfig.inverted(DriveConstants.frontLeftTurnInverted);  break;
            case FRONT_RIGHT: turnConfig.inverted(DriveConstants.frontRightTurnInverted); break;
            case BACK_LEFT  : turnConfig.inverted(DriveConstants.backLeftTurnInverted);   break;
            case BACK_RIGHT : turnConfig.inverted(DriveConstants.backRightTurnInverted);  break;
            default -> turnConfig.inverted(false);
        }

        //idleMode is Brake
        //stay at position when stopped
        //set the smart current limit to avoid going over what the motor can handle
        //voltage compensation = 12 because working with 12v battery
        turnConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveConstants.turnMotorCurrentLimit)
            .voltageCompensation(12);

        
        //accounts for gearing, this allows us to convert from rotations to radians and rotations/min to radians/sec
        turnConfig
            .absoluteEncoder
            .positionConversionFactor(DriveConstants.turnEncoderPositionFactor)
            .velocityConversionFactor(DriveConstants.turnEncoderVelocityFactor)
            .setSparkMaxDataPortConfig()
            .averageDepth(2);

        
        //configure how often the turnmotor receives/uses signals
        //we set the encoder speed to the a calculation
        //set the velocity to always give output
        //set every other periodic function in the motor to 20ms the standard
        turnConfig
            .signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs(
                (int) (1000.0 / DriveConstants.odometryFrequency)) 
            //as of now constant doesnt exist if anyone adds it in pls remove this comment just do it when odometry is added
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        //configure turn motor
        turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



        //DRIVE
        //some of the things are the exact same
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        switch (module) {
            case FRONT_LEFT -> driveConfig.inverted(DriveConstants.frontLeftDriveInverted);
            case FRONT_RIGHT -> driveConfig.inverted(DriveConstants.frontRightDriveInverted);
            case BACK_LEFT -> driveConfig.inverted(DriveConstants.backLeftDriveInverted);
            case BACK_RIGHT -> driveConfig.inverted(DriveConstants.backRightDriveInverted);
            default -> driveConfig.inverted(false);
        }

        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveConstants.driveMotorCurrentLimit)
            .voltageCompensation(12.0);

        
        driveConfig
            .encoder
            .positionConversionFactor(DriveConstants.driveEncoderPositionFactor)
            .velocityConversionFactor(DriveConstants.driveEncoderVelocityFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        
        driveConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        driveSpark.configure(
            driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //reset drive encoder
        driveEncoder.setPosition(0.0);

        //make odometry queues here the next person can do this
        //No odometry for now (I don't want to deal with it rn)
    }

    //pretty sure you're not supposed to do it like this
    @Override
    public void updateInputs(ModuleIOInputs inputs) { 
        inputs.driveConnected = driveDebouncer.calculate(driveSpark.getLastError() == REVLibError.kOk);
        if(inputs.driveConnected) {
            inputs.drivePosition_Rad = driveEncoder.getPosition().get();
            inputs.driveVelocity_RadPerSec = driveEncoder.getVelocity().get();
            inputs.driveAppliedVoltage_Volts = driveSpark.getBusVoltage().get() * driveSpark.getAppliedOutput().get();
            inputs.driveCurrent_Amps = driveSpark.getOutputCurrent().get();
        }

        inputs. turnConnected = turnDebouncer.calculate(turnSpark.getLastError() == REVLibError.kOk);
        inputs.turnPosition = Rotation2d.fromRotations(absoluteEncoder.get()).minus(rotationOffset).plus(Rotation2d.kPi);
        inputs.turnEncoder_Volts = absoluteAnalogInput.getVoltage();
        if(inputs.turnConnected) {
            inputs.turnVelocity_RadPerSec = turnEncoder.getVelocity().get();
            inputs.turnAppliedVoltage_Volts = turnSpark.getBusVoltage().get() * turnSpark.getAppliedOutput().get();
            inputs.turnCurrent_Amps = turnSpark.getOutputCurrent().get();
        }
    }

    @Override
    //voltage
    public void setDriveVoltage(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnVoltage(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velo) {
        double ffvolts = DriveConstants.driveKs * Math.signum(velo)
            + DriveConstants.driveKv * velo;
        driveClosedLoopController.setSetpoint(
            velo, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffvolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnAngle(Rotation2d rotation) {
        double setPoint =
            MathUtil.inputModulus(
                rotation.plus(rotationOffset).getRadians(),
                DriveConstants.turnPIDMinInput,
                DriveConstants.turnPIDMaxInput);
        double volts =
            turnPID.calculate(
                (Rotation2d.fromRotations(absoluteEncoder.get()).plus(Rotation2d.kPi)).getRadians(),
                setPoint);

        setTurnVoltage(volts);
    }
    
    public double getRawRadians() {
        return Rotation2d.fromRotations(absoluteEncoder.get()).getRadians();
    }
}
