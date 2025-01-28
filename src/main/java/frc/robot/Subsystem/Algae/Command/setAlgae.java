// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Algae.Command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Algae.Algae;

public class setAlgae extends Command {
  public Algae algae;
  public double setPoint;
  public double speed;

  public setAlgae(Algae algae, double setPoint, double speed) {
    this.algae = algae;
    this.setPoint = setPoint;
    this.speed = speed;
  }

  @Override
  public void execute() {
    algae.setPivot(setPoint);
    algae.setRollerSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.setRollerSpeed(0);
    algae.setPivot( 0);
  }
}
