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
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorIOSparkMax implements ElevatorIO {
    
    ElevatorInputsAutoLogged inputs;

    public SparkMax sparkyLeft;
    public SparkMax sparkyRight;

    private SparkMaxConfig sparkyLeftConfig = new SparkMaxConfig();
    private SparkMaxConfig sparkyRightConfig = new SparkMaxConfig();

    public ElevatorIOSparkMax(){

    inputs = new ElevatorInputsAutoLogged();
    sparkyLeft = new SparkMax(ElevatorConstants.sparkyLeft, MotorType.kBrushless);
    sparkyRight = new SparkMax(ElevatorConstants.sparkyRight, MotorType.kBrushless);

    sparkyLeftConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(10);  // I did some math there is 0.6 resistance and that multipy by 10 is 6 volts.
    sparkyLeftConfig.encoder
      .positionConversionFactor(6)
      .velocityConversionFactor(1.0/10.0);

    sparkyRightConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(10);
    sparkyRightConfig.encoder
      .positionConversionFactor(6)
      .velocityConversionFactor(1.0/10.0);

    sparkyLeft.configure(sparkyLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkyRight.configure(sparkyRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.position = sparkyLeft.getEncoder().getPosition();
    inputs.encoderVelocity = sparkyLeft.getEncoder().getVelocity();

    inputs.leftAppliedVolts = sparkyLeft.getAppliedOutput();
    inputs.rightAppliedVolts = sparkyRight.getAppliedOutput();

    inputs.leftSparkTemp = sparkyLeft.getMotorTemperature();
    inputs.rightSparkTemp = sparkyRight.getMotorTemperature();
  }

  @Override
  public void setElevator(double setPoint) {

    if(inputs.position == 60){  //turns motors off if it reaches 60 testing hardstop
      sparkyLeft.set(0);
      sparkyRight.set(0);
    }

    sparkyLeft.set(setPoint);
    sparkyRight.set(setPoint);

    Logger.recordOutput("Encoder", sparkyLeft.getEncoder().getPosition());

    Logger.recordOutput("Sparky Left Output", setPoint);
    Logger.recordOutput("Sparky Right Output", setPoint);
  }
  
  @Override
  public void setManualSpeed(double volts) {
    sparkyLeft.set(volts);
    sparkyRight.set(volts);

    Logger.recordOutput("left Volts", volts);
    Logger.recordOutput("right Volts", volts);
  }

  @Override
  public void resetEncoder(double position) {
   inputs.position = 0.0;
  }
}


