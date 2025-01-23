// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorIOSparkMax implements ElevatorIO {
    
    ElevatorInputsAutoLogged inputs;

    public SparkMax sparkyLeft;
    public SparkMax sparkyRight;

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController feedbackLeft;
    private final SparkClosedLoopController feedbackRight;

    public ElevatorIOSparkMax(){

    inputs = new ElevatorInputsAutoLogged();
    sparkyLeft = new SparkMax(30, MotorType.kBrushless);
    sparkyRight = new SparkMax( 31,MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kCoast);
    config.encoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);
    config.closedLoop
        .p(ElevatorConstants.kP)
        .i(ElevatorConstants.kI)
        .d(ElevatorConstants.kD);

    sparkyLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(sparkyLeft, true);
    sparkyRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = sparkyLeft.getEncoder();
    feedbackLeft = sparkyLeft.getClosedLoopController();
    feedbackRight = sparkyRight.getClosedLoopController();
    encoder.setPosition(0);
    }


  @Override

  public void updateInputs(ElevatorInputs inputs) {
    Logger.recordOutput("Elevator/Encoder", encoder.getVelocity());

    inputs.position = encoder.getVelocity();
    inputs.encoderVelocity = sparkyLeft.getEncoder().getVelocity();

    inputs.leftAppliedVolts = sparkyLeft.getAppliedOutput();
    inputs.rightAppliedVolts = sparkyRight.getAppliedOutput();

    inputs.leftSparkTemp = sparkyLeft.getMotorTemperature();
    inputs.rightSparkTemp = sparkyRight.getMotorTemperature();
  }

  @Override
  public void setElevator(double setPoint, double voltage) {
    feedbackLeft.setReference(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, voltage);
    Logger.recordOutput("Elevator/Sparky Left Output", setPoint);
    Logger.recordOutput("Elevator/Sparky Right Output", setPoint);
  }
  // @Override
  // public void resetEncoder(double position) {
  //   encoder.setPosition(0.0);
  // }
}


