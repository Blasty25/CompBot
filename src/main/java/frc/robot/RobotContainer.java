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
import frc.robot.Subsystem.Elevator.Commands.runL1;
import frc.robot.Subsystem.Elevator.Commands.runL2;
import frc.robot.Subsystem.Elevator.Commands.runL3;
import frc.robot.Subsystem.Elevator.Commands.runL4;
import frc.robot.Subsystem.Elevator.Commands.runSetPoint;
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
        controller.leftBumper().whileTrue(new runSetPoint(elevator , 360));
        controller.povDown().whileTrue(new runL1(elevator, 6480));
        controller.povLeft().whileTrue(new runL2(elevator, 11480.33));
        controller.povRight().whileTrue(new runL3(elevator, 17149.61));
        controller.povUp().whileTrue(new runL4(elevator, 25936.99));

        
        // elevator.setDefaultCommand(
        //     elevator.elevatorJoystick(
        //       () -> controller.getRightY()
        //     )
        // );
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
