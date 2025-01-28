// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Corral.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Corral.Corral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setCorral extends Command {
  public Corral corral;
  private double volts;

  public setCorral(Corral corral, double volts) {
    this.corral = corral;
    this.volts = volts;
  }

  @Override
  public void execute() {
    corral.setSpeed(volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    corral.setSpeed(0);
  }

}
