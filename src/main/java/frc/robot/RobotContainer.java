// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.DriveConstants.ModuleOrientation;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Module.ModuleIOSpark;

public class RobotContainer {
  
  public final Swerve swerve;

  CommandXboxController primaryController = new CommandXboxController(0);


  public RobotContainer() {
    swerve = new Swerve(
      new ModuleIOSpark(ModuleOrientation.FRONT_LEFT), 
      new ModuleIOSpark(ModuleOrientation.BACK_LEFT), 
      new ModuleIOSpark(ModuleOrientation.FRONT_RIGHT), 
      new ModuleIOSpark(ModuleOrientation.BACK_RIGHT),
      primaryController
      );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
