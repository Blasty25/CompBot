// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  ElevatorInputsAutoLogged inputs;
  private ElevatorIO io;

  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1); //Inches/sec

  private final ProfiledPIDController m_controller = new ProfiledPIDController(
      ElevatorConstants.kP,
      ElevatorConstants.kI,
      ElevatorConstants.kP,
      constraints);

  private final ElevatorFeedforward m_elevator = new ElevatorFeedforward(
      ElevatorConstants.kS,
      ElevatorConstants.kG,
      ElevatorConstants.kV,
      ElevatorConstants.kA);


  private double setpoint = Units.inchesToMeters(44);

  public ElevatorSubsystem(ElevatorIO height) {

    inputs = new ElevatorInputsAutoLogged();
    this.io = height;
  }


  public void setManualSpeed() {

    m_controller.setGoal(setpoint);

    double pidOutput = m_controller.calculate(inputs.elevatorPosition);
    double feedforward = m_elevator.calculate(m_controller.getSetpoint().velocity);
    
    io.setManualSpeed(pidOutput + feedforward);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs); 
  }

}


/*    TrapezoidProfile.State targetState = new TrapezoidProfile.State(Units.inchesToMeters(44), 0.0);
    futureProfileState = profile.calculate(0.02, profileState, targetState);

    double feedForward = m_elevator.calculate(futureProfileState.velocity, (futureProfileState.velocity - profileState.velocity) /0.02);
    double feedBack = m_controller.calculate(inputs.elevatorPosition, futureProfileState.position); */
