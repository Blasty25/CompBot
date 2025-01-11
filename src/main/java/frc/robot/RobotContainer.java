// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystem.Elevator.ElevatorIOSim;
import frc.robot.Subsystem.Elevator.ElevatorIOSparkMax;
import frc.robot.Subsystem.Elevator.ElevatorSubsystem;
import frc.robot.Subsystem.Maple_Sim.Maple_Drive.MapleSwerve;
import frc.robot.Subsystem.Swerve.Drive;
import frc.robot.Subsystem.Swerve.DriveCommands;
import frc.robot.Subsystem.Swerve.GyroIOPigeon2;
import frc.robot.Subsystem.Swerve.ModuleIOSim;
import frc.robot.Subsystem.Swerve.ModuleIOSparkMax;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);
  private Drive drive;
  private ElevatorSubsystem elevator;
  private ElevatorIOSim elevatorSim = new ElevatorIOSim();
  private ElevatorIOSparkMax realElevator = new ElevatorIOSparkMax();

  public RobotContainer() {

    if (Robot.isReal()) {
      drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOSparkMax(0),
          new ModuleIOSparkMax(1),
          new ModuleIOSparkMax(2),
          new ModuleIOSparkMax(3));
      elevator = new ElevatorSubsystem(realElevator);
    } else {
      drive = new Drive(
        new GyroIOPigeon2(),
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim());
      elevator = new ElevatorSubsystem(elevatorSim);      
    }
    configureBindings();
  }

  private void configureBindings() {

    switch (Constants.simState) {
      case Advantage:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> -controller.getRightX()));
                 controller.povUp().onTrue(elevator.elevatorUp());
                 controller.povDown().onTrue(elevator.elevatorDown());
        elevator.setDefaultCommand(
            elevator.elevatorJoystick(
              () -> controller.getRightY()
            )
        );
        break;
      case Maple:
        // mapleSimSwerve.drive(T, 2, false, true);
        break;
    }

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
