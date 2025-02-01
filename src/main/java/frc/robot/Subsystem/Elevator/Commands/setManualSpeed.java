// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Elevator.ElevatorInputsAutoLogged;
import frc.robot.Subsystem.Elevator.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setManualSpeed extends Command {
  public ElevatorSubsystem elevator;
  private ElevatorInputsAutoLogged inputs;

  /** Creates a new setManualSpeed. */
  public setManualSpeed(ElevatorSubsystem elevator) {
    inputs = new ElevatorInputsAutoLogged();
    this.elevator = elevator;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setManualSpeed();
  }
  // Called once the command ends or is interrupted.
  @Override
  public boolean isFinished() {
    return Math.abs(inputs.elevatorPosition - 44.0) < 1.0;
  }
}
