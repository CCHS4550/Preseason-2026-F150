// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.DriveConstants.ModuleOrientation;
import frc.robot.Subsystems.Elevator.ElevatorIOCTRE;
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
    //configureBindings();
  }

  private void configureBindings() {
    primaryController
      .leftBumper()
      .onTrue(new InstantCommand(() -> { grabber.setWantedState(Grabber.WantedState.INTAKE); }))
      .onFalse(new InstantCommand(() -> { grabber.setWantedState(Grabber.WantedState.IDLE); }));

    primaryController
      .rightBumper()
      .onTrue(new InstantCommand(() -> { grabber.setWantedState(Grabber.WantedState.OUTTAKE); }))
      .onFalse(new InstantCommand(() -> { grabber.setWantedState(Grabber.WantedState.IDLE); }));
    
    primaryController
      .a()
      .onTrue(new InstantCommand(() -> { elevator.setWantedState(Elevator.WantedState.DRIVER_STATION); }));

    primaryController
      .b()
      .onTrue(new InstantCommand(() -> { elevator.setWantedState(Elevator.WantedState.LEVEL_1); }));

    primaryController
      .x()
      .onTrue(new InstantCommand(() -> { elevator.setWantedState(Elevator.WantedState.LEVEL_2); }));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
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
