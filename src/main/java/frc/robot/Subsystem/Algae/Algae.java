// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Algae;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
  SparkMax rollerAlgae;
  SparkMax pivotAlgae;
  SparkMaxConfig config = new SparkMaxConfig();
  PIDController pid = new PIDController(1, 0, 0);

  /** Creates a new Algae. */
  public Algae() {
    rollerAlgae = new SparkMax(11, MotorType.kBrushless);
    pivotAlgae = new SparkMax(12, MotorType.kBrushless);

    config
    .inverted(false)
    .idleMode(IdleMode.kCoast);
    config.encoder
    .positionConversionFactor((5/1)*(50/50));
  }

  public void setPivot(double speed){
    pivotAlgae.set(pid.calculate(pivotAlgae.getEncoder().getPosition(), speed));
  }

  public void setRollerSpeed(double speed){
    rollerAlgae.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
