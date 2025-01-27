// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystem.Elevator.ElevatorIOSim;
import frc.robot.Subsystem.Elevator.ElevatorIOSparkMax;
import frc.robot.Subsystem.Elevator.ElevatorSubsystem;
import frc.robot.Subsystem.Elevator.Commands.setManualSpeed;
import frc.robot.Subsystem.Maple_Sim.Maple_Drive.MapleSwerve;
import frc.robot.Subsystem.Swerve.Drive;
import frc.robot.Subsystem.Swerve.GyroIOPigeon2;
import frc.robot.Subsystem.Swerve.ModuleIOSim;
import frc.robot.Subsystem.Swerve.ModuleIOSpark;
import frc.robot.Subsystem.Swerve.Commands.DriveCommands;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);
  // private Drive drive;
  private ElevatorSubsystem elevator;
  private ElevatorIOSim elevatorSim = new ElevatorIOSim();
  private ElevatorIOSparkMax realElevator = new ElevatorIOSparkMax();
  // private MapleSwerve swerve = new MapleSwerve();

  public RobotContainer() {

    if (Robot.isReal()) {
      // drive = new Drive(
      //     new GyroIOPigeon2(),
      //     new ModuleIOSpark(0),
      //     new ModuleIOSpark(1),
      //     new ModuleIOSpark(2),
      //     new ModuleIOSpark(3));
      elevator = new ElevatorSubsystem(realElevator);
    } else {
      // drive = new Drive(
      //     new GyroIOPigeon2(),
      //     new ModuleIOSim(),
      //     new ModuleIOSim(),
      //     new ModuleIOSim(),
      //     new ModuleIOSim());
      elevator = new ElevatorSubsystem(elevatorSim);
      // swerve = new MapleSwerve();
    }
    configureBindings();
  }

  private void configureBindings() {

        // drive.setDefaultCommand(
        //     DriveCommands.joystickDrive(
        //         drive,
        //         () -> controller.getLeftY(),
        //         () -> controller.getLeftX(),
        //         () -> -controller.getRightX()));
        controller.a().onTrue(new setManualSpeed(elevator));
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}