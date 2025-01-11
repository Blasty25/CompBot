// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Elevator.ElevatorSubsystem;

public class RunSetElevator extends Command {
  ElevatorSubsystem elevator;
  public final double setPoint;

  public RunSetElevator(ElevatorSubsystem elevator, double setPoint) {
    this.elevator = elevator;
    this.setPoint = setPoint;
  }

  @Override
  public void execute() {
    elevator.setElevator(setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setElevator(0);
  }
}
