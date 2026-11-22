// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.DriveConstants.ModuleOrientation;
import frc.robot.Subsystems.Elevator.ElevatorIOCTRE;
import frc.robot.Subsystems.Elevator.Elevator.WantedState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Grabber.Grabber;
import frc.robot.Subsystems.Grabber.GrabberIOCTRE;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Gyro.GyroIOPigeon;
import frc.robot.Subsystems.Swerve.Module.ModuleIOSpark;

public class RobotContainer {
  
  public final Swerve swerve;
  public final Grabber grabber;
  public final Elevator elevator;

  CommandXboxController primaryController = new CommandXboxController(0);

  public RobotContainer() {
    swerve = new Swerve(
      new GyroIOPigeon(),
      new ModuleIOSpark(ModuleOrientation.FRONT_LEFT), 
      new ModuleIOSpark(ModuleOrientation.BACK_LEFT), 
      new ModuleIOSpark(ModuleOrientation.FRONT_RIGHT), 
      new ModuleIOSpark(ModuleOrientation.BACK_RIGHT),
      primaryController
      );
    grabber = new Grabber(
      new GrabberIOCTRE()
    );
    elevator = new Elevator(
      new ElevatorIOCTRE()
    );
    configureBindings();
  }

  private void configureBindings() {
    primaryController
      .rightBumper()
      .onTrue(new InstantCommand(() -> { grabber.setWantedState(Grabber.WantedState.INTAKE); }))
      .onFalse(new InstantCommand(() -> { grabber.setWantedState(Grabber.WantedState.IDLE); }));

    primaryController
      .leftBumper()
      .onTrue(new InstantCommand(() -> { grabber.setWantedState(Grabber.WantedState.OUTTAKE); }))
      .onFalse(new InstantCommand(() -> { grabber.setWantedState(Grabber.WantedState.IDLE); }));
    
    primaryController
      .a()
      .onTrue(new InstantCommand(() -> { elevator.setWantedState(Elevator.WantedState.OPENLOOP); }))
      .whileTrue(new InstantCommand(() -> { elevator.setVoltage(4.5); }))
      .onFalse(new InstantCommand(() -> { elevator.setWantedState(Elevator.WantedState.IDLE); }));

    primaryController
      .b()
      .onTrue(new InstantCommand(() -> { elevator.setWantedState(Elevator.WantedState.OPENLOOP); }))
      .whileTrue(new InstantCommand(() -> { elevator.setVoltage(-4.5); }))
      .onFalse(new InstantCommand(() -> { elevator.setWantedState(Elevator.WantedState.IDLE); }));

    // primaryController
    //   .x()
    //   .onTrue(new InstantCommand(() -> { elevator.setWantedState(Elevator.WantedState.LEVEL_2); }));
    primaryController
      .y()
      .onTrue(new InstantCommand(() -> { swerve.setWantedState(Swerve.WantedState.IDLE);}));
  }

  public Command getAutonomousCommand() {
    return Commands.parallel(new InstantCommand(() -> {grabber.setWantedState(Grabber.WantedState.INTAKE);}), new WaitCommand(2.0))
      .andThen(new InstantCommand(() -> {grabber.setWantedState(Grabber.WantedState.IDLE);}))
      .andThen(Commands.parallel(new InstantCommand(() -> {swerve.setTargetChassisSpeeds(new ChassisSpeeds(0.0, -0.5, 0.0));}), new WaitCommand(1.0)))
      .andThen(new InstantCommand(() -> {swerve.setWantedState(Swerve.WantedState.IDLE);}));
  }

  public Swerve getSwerveSubsystem() {
    return swerve;
  }

  public void setAllIdle() {
    swerve.setWantedState(Swerve.WantedState.IDLE);
    elevator.setWantedState(Elevator.WantedState.IDLE);
    grabber.setWantedState(Grabber.WantedState.IDLE);
  }
}
