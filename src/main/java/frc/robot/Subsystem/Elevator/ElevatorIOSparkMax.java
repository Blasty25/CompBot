// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystem.Elevator.ElevatorIO.ElevatorInputs;

/** Add your docs here. */
public class ElevatorIOSparkMax implements ElevatorIO {
    
    ElevatorInputsAutoLogged inputs;

    private final ProfiledPIDController m_controller = new ProfiledPIDController(
    ElevatorConstants.kP,
    ElevatorConstants.kI,
    ElevatorConstants.kP,
    new TrapezoidProfile.Constraints(2.45, 2.45));

    private SparkMax sparkyLeft;
    private SparkMax sparkyRight;

    private SparkMaxConfig sparkyLeftConfig = new SparkMaxConfig();
    private SparkMaxConfig sparkyRightConfig = new SparkMaxConfig();

    public ElevatorIOSparkMax(){
    inputs = new ElevatorInputsAutoLogged();
    sparkyLeft = new SparkMax(ElevatorConstants.sparkyLeft, MotorType.kBrushless);
    sparkyRight = new SparkMax(ElevatorConstants.sparkyRight, MotorType.kBrushless);


    //Configuring the Encoder on the Spark Max is not Avliable until complete documentation
    //https://docs.revrobotics.com/brushless/revlib/revlib-overview/migrating-to-revlib-2025
    //Using a external encoder for now
    sparkyLeftConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast);

    sparkyRightConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast);

    sparkyLeft.configure(sparkyLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkyRight.configure(sparkyRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

      @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.leftAppliedVolts = sparkyLeft.getAppliedOutput();
    inputs.rightAppliedVolts = sparkyRight.getAppliedOutput();
    inputs.leftSparkTemp = sparkyLeft.getMotorTemperature();
    inputs.rightSparkTemp = sparkyRight.getMotorTemperature();
  }

  @Override
  public void setElevator(double setPoint) {

    m_controller.setGoal(setPoint);

    double pidOutput = m_controller.calculate(sparkyLeft.getEncoder().getPosition());
    sparkyLeft.setVoltage(pidOutput);
    sparkyRight.setVoltage(pidOutput);
  }

    @Override
  public void setManualSpeed(double volts) {
    sparkyLeft.set(volts);
    sparkyRight.set(volts);
    Logger.recordOutput("left Volts", volts);
    Logger.recordOutput("right Volts", volts);
  }

}


