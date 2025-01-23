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
    new TrapezoidProfile.Constraints(1, 1));

  private final ElevatorFeedforward m_elevator = new ElevatorFeedforward(
    ElevatorConstants.kS,
    ElevatorConstants.kG,
    ElevatorConstants.kV,
    ElevatorConstants.kA
  );


  private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 1));
  private TrapezoidProfile.State profileState = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State futureProfileState = new TrapezoidProfile.State(0, 0);




  public ElevatorSubsystem(ElevatorIO height){
    inputs = new ElevatorInputsAutoLogged();
    this.io =  height;
  }

  public void setElevator(double setPoint) {
    futureProfileState = profile.calculate(0.02, profileState, new TrapezoidProfile.State(setPoint, 0.0)); 
    io.setElevator(profileState.position, m_elevator.calculate(profileState.velocity, futureProfileState.velocity));
    profileState = futureProfileState;
  }

  public void setManualSpeed(double position){
    io.setManualSpeed(position);
  }

  // public Command resetEncoder(){
  //   return new RunCommand(()->{
  //     io.resetEncoder();
  //   }, this);
  //   }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs); 
  }

}
