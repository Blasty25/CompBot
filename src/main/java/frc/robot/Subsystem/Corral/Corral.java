// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Corral;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Corral extends SubsystemBase {
  TalonSRX leftRollar = new TalonSRX(21);
  TalonSRX rightRollar = new TalonSRX(22);

  public Corral() {
    leftRollar.setInverted(false);
    rightRollar.setInverted(false);

    leftRollar.enableCurrentLimit(true);
    rightRollar.enableCurrentLimit(true);
  }

  public void setSpeed(double volts){
    leftRollar.set(ControlMode.PercentOutput, volts);
    rightRollar.set(ControlMode.PercentOutput, volts);
  }

  public Command shooter(DoubleSupplier speed){
    return new RunCommand(()->{
      double volts = speed.getAsDouble();
      setSpeed(volts);
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
