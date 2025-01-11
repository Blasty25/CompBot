// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator.Commands;

import org.dyn4j.exception.ArgumentNullException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Elevator.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunSpeed extends Command {
  ElevatorSubsystem elevator;
  public double volts;

  /** Creates a new RunSpeed. */
  public RunSpeed(ElevatorSubsystem elevator, double volts) {
    this.elevator = elevator;
    this.volts = volts;
  }
  @Override
  public void execute() {
    elevator.setManualSpeed(volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setManualSpeed(0);
  }

}
