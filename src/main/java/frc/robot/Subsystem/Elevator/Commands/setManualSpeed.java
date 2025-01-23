// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator.Commands;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Elevator.ElevatorIOSparkMax;
import frc.robot.Subsystem.Elevator.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setManualSpeed extends Command {
  public ElevatorIOSparkMax realElevator;
  public ElevatorSubsystem elevator;
  SparkMax left;
  SparkMax right;
  /** Creates a new setManualSpeed. */
  public setManualSpeed(ElevatorIOSparkMax realElevator) {
    this.realElevator = realElevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.left = realElevator.sparkyLeft;
    this.right = realElevator.sparkyRight;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    left.getClosedLoopController().setReference(3.0, ControlType.kPosition);
    right.getClosedLoopController().setReference(3.0, ControlType.kPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
