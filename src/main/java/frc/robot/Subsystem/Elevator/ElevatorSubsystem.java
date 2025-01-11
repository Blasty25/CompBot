// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  ElevatorInputsAutoLogged inputs;
  private ElevatorIO io;

  public ElevatorSubsystem(ElevatorIO height){
    inputs = new ElevatorInputsAutoLogged();
    this.io =  height;
  }

  
  public void setManualSpeed(double volts) {
   io.setManualSpeed(volts);
  }

  public Command setDutyCycle(DoubleSupplier volts) {
    return this.run(() -> {
      setManualSpeed(volts.getAsDouble());
    });
  }

  public Command elevatorJoystick(DoubleSupplier volts){
    return new RunCommand(()->{
      double value = volts.getAsDouble();
      MathUtil.applyDeadband(value, 0.3);
      setManualSpeed(value);
    }, this);
  }

  public Command elevatorUp(){
    return new RunCommand(()->{
      double upVolts = 5;
      this.setManualSpeed(upVolts);
    },this);
  }

  public Command elevatorDown(){
    return new RunCommand(() -> {
      double downVolts = -5;
      this.setManualSpeed(downVolts);
  }, this);
  }

  @Override
  public void periodic() {
    Logger.processInputs("Elevator", inputs);

    SmartDashboard.putData("Elevator 1 volt", setDutyCycle(() -> 1));
    SmartDashboard.putData("Elevator -1 volt", setDutyCycle(() -> -1));
  }
}
