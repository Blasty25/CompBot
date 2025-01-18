// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
public class ElevatorSubsystem extends SubsystemBase{
  /** Creates a new ElevatorSubsystem. */
  ElevatorInputsAutoLogged inputs;
  private ElevatorIO io;

  private final ProfiledPIDController m_controller = new ProfiledPIDController(
    ElevatorConstants.kP,
    ElevatorConstants.kI,
    ElevatorConstants.kP,
    new TrapezoidProfile.Constraints(2.45, 2.45));

  private final ElevatorFeedforward m_elevator = new ElevatorFeedforward(
    ElevatorConstants.kS,
    ElevatorConstants.kG,
    ElevatorConstants.kV,
    ElevatorConstants.kA
  );




  public ElevatorSubsystem(ElevatorIO height){
    inputs = new ElevatorInputsAutoLogged();
    this.io =  height;
  }

  public void setElevator(double setPoint) {
    double feedFowardOutput = m_elevator.calculate(inputs.encoderVelocity);
    double feedbackOutput = m_controller.calculate(inputs.position, setPoint);
    setPoint = feedFowardOutput + feedbackOutput;
    io.setElevator(setPoint);
    Logger.recordOutput("Spark Max Encoder", inputs.position);
  }

  public Command resetEncoder(){
    return new RunCommand(()->{
      io.resetEncoder();
    }, this);
    }

  @Override
  public void periodic() {
    Logger.processInputs("Elevator", inputs); 
  }

}
