// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Elevator.ElevatorIOSparkMax;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class runL3 extends Command {
  ElevatorIOSparkMax elevator;
  private double setpoint;
  /** Creates a new runSetPoint. */
  public runL3(ElevatorIOSparkMax elevator, double setpoint) {
    this.elevator = elevator;
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {
    elevator.setElevator(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setElevator(360);
  }
}
