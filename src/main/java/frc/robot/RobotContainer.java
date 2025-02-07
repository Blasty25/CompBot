// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import org.ironmaple.simulation.SimulatedArena;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystem.Algae.Algae;
import frc.robot.Subsystem.Algae.Command.setAlgae;
import frc.robot.Subsystem.Corral.Corral;
import frc.robot.Subsystem.Corral.Commands.setCorral;
import frc.robot.Subsystem.Swerve.Drive;
import frc.robot.Subsystem.Swerve.GyroIOPigeon2;
import frc.robot.Subsystem.Swerve.ModuleIOSim;
import frc.robot.Subsystem.Swerve.ModuleIOSpark;
import frc.robot.Subsystem.Swerve.Commands.DriveCommands;
import frc.robot.Subsystem.elevator.Elevator;
import frc.robot.Subsystem.elevator.ElevatorIOReal;
import frc.robot.Subsystem.elevator.ElevatorIOSim;

public class RobotContainer {
  private CommandXboxController controller = new CommandXboxController(0);
  private Drive drive;
  private Algae algae;
  private Corral corral;
  private Elevator elevator;

  public RobotContainer() {

    if (Robot.isReal()) {
      // drive = new Drive(
      //     new GyroIOPigeon2(),
      //     new ModuleIOSpark(0),
      //     new ModuleIOSpark(1),
      //     new ModuleIOSpark(2),
      //     new ModuleIOSpark(3));
      algae = new Algae();
      corral = new Corral();
      elevator = new Elevator(new ElevatorIOReal(30,21));
    } else {
      drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim());
      corral = new Corral();
      elevator = new Elevator(new ElevatorIOSim());
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
        controller.rightTrigger().whileTrue(new setCorral(corral, 5));
        // controller.y().whileTrue(new setAlgae(algae, 1, 1));
        // controller.b().whileTrue(new setAlgae(algae, -1, -1));
        controller.a().toggleOnTrue(elevator.nextLevel());

    }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Path");
    }
}