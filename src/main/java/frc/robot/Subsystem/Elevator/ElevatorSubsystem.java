// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Elevator.ElevatorIOSparkMax;
public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  ElevatorInputsAutoLogged inputs;
  private ElevatorIO io;
  private final MechanismLigament2d m_elevator;
  private double elevatorMin = 0.5;
  private Encoder simEncoder = new Encoder(0, 1);
  private ElevatorIOSparkMax elevatorSet;


  public ElevatorSubsystem(ElevatorIO height){
  
    simEncoder.setDistancePerPulse(0.01);
    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("elevator", 2, 0);
    m_elevator = root.append(new MechanismLigament2d("elevator", elevatorMin, 90));
    SmartDashboard.putData("Mech 2d", mech);
    inputs = new ElevatorInputsAutoLogged();
    this.io =  height;
  }

  
  public void setManualSpeed(double volts) {
   io.setManualSpeed(volts);
   m_elevator.setLength(elevatorMin + simEncoder.getDistance());
  }

  // public Command setDutyCycle(DoubleSupplier volts) {
  //   return this.run(() -> {
  //     setManualSpeed(volts.getAsDouble());
  //   });
  // }

  public Command elevatorJoystick(DoubleSupplier volts){
    return new RunCommand(()->{
      double value = volts.getAsDouble();
      MathUtil.applyDeadband(value, 0.3);
      elevatorSet.setElevator(value);
    }, this);
  }

  public Command elevatorUp(){
    return new RunCommand(()->{
      double upVolts = 1;
      this.setManualSpeed(upVolts);
    },this);
  }

  public Command elevatorDown(){
    return new RunCommand(() -> {
      double downVolts = -1;
      elevatorSet.setElevator(downVolts);
  }, this);
  }  


  @Override
  public void periodic() {

    Logger.processInputs("Elevator", inputs);
  }
}
