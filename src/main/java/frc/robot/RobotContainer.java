// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystem.Elevator.ElevatorSubsystem;
import frc.robot.Subsystem.Elevator.Commands.RunSetElevator;
import frc.robot.Subsystem.Maple_Sim.Maple_Drive.MapleSimSwerve;
import frc.robot.Subsystem.Swerve.Drive;
import frc.robot.Subsystem.Swerve.DriveCommands;
import frc.robot.Subsystem.Swerve.GyroIOPigeon2;
import frc.robot.Subsystem.Swerve.ModuleIOSparkMax;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);
  private Drive drive = null;
  private MapleSimSwerve mapleSimSwerve = null;
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // drive = new Drive(
        //     new GyroIOPigeon2(),
        //     new ModuleIOSparkMax(0),
        //     new ModuleIOSparkMax(1),
        //     new ModuleIOSparkMax(2),
        //     new ModuleIOSparkMax(3));
        elevatorSubsystem = new ElevatorSubsystem();
        break;
      case SIM:
        mapleSimSwerve = new MapleSimSwerve();
        break;

    }
    configureBindings();
  }

  private void configureBindings() {
    // if (drive != null) {
    //   drive.setDefaultCommand(
    //       DriveCommands.joystickDrive(
    //           drive,
    //           () -> controller.getLeftY(),
    //           () -> controller.getLeftX(),
    //           () -> -controller.getRightX()));
    //    controller.y().whileTrue(new RunSetElevator(elevatorSubsystem, 15));
    //    controller.a().whileTrue(new RunSetElevator(elevatorSubsystem, -15));
    // if (mapleSimSwerve != null) {
    //     mapleSimSwerve.setDefaultCommand(
    //         DriveCommands.joystickDrive(
    //             drive,
    //             () -> controller.getLeftY(),
    //             () -> controller.getLeftX(),
    //             () -> -controller.getRightX()));
    //   }
    // }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
