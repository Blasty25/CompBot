// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystem.Maple_Sim.Maple_Drive.MapleSimSwerve;
import frc.robot.Subsystem.Swerve.Drive;
import frc.robot.Subsystem.Swerve.DriveCommands;
import frc.robot.Subsystem.Swerve.GyroIOPigeon2;
import frc.robot.Subsystem.Swerve.ModuleIOSparkMax;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);
  private Drive drive;
  private MapleSimSwerve mapleSimSwerve;

  public RobotContainer() {
    if(Robot.isReal()) {
     drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOSparkMax(0),
          new ModuleIOSparkMax(1),
          new ModuleIOSparkMax(2),
          new ModuleIOSparkMax(3));
    }else{
      MapleSimSwerve mapleSwerve = new MapleSimSwerve();
    }
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
      DriveCommands.joystickDrive(
        drive,
        () -> controller.getLeftY(),
        () -> controller.getLeftX(),
        () -> -controller.getRightX()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
