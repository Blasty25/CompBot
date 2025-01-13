// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import javax.xml.stream.events.EndDocument;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        .idleMode(IdleMode.kCoast);
    sparkyLeftConfig.encoder
      .positionConversionFactor(6)
      .velocityConversionFactor(1.0/10.0);

    sparkyRightConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast);
    sparkyRightConfig.encoder
      .positionConversionFactor(6)
      .velocityConversionFactor(1.0/10.0);

    sparkyLeft.configure(sparkyLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkyRight.configure(sparkyRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
    }
    
  public double leftPosition = sparkyLeft.getEncoder().getPosition();
  public double rightPosition = sparkyRight.getEncoder().getPosition();


  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.leftAppliedVolts = sparkyLeft.getAppliedOutput();
    inputs.rightAppliedVolts = sparkyRight.getAppliedOutput();
    inputs.leftSparkTemp = sparkyLeft.getMotorTemperature();
    inputs.rightSparkTemp = sparkyRight.getMotorTemperature();
  }

  @Override
  public void setElevator(double setPoint) {
    sparkyLeft.set(setPoint);
    sparkyRight.set(setPoint);

    Logger.recordOutput("Left Encoder", sparkyLeft.getEncoder().getPosition());
    Logger.recordOutput("Right Encoder", sparkyRight.getEncoder().getPosition());
    Logger.recordOutput("Sparky Left Output", setPoint);
    Logger.recordOutput("Sparky Right Output", setPoint);
  }
  
  @Override
  public void setManualSpeed(double volts) {
    sparkyLeft.set(volts);
    Logger.recordOutput("left Volts", volts);
    Logger.recordOutput("right Volts", volts);
  }

  @Override
  public void resetEncoder(double position) {
   sparkyLeft.getEncoder().setPosition(0);
   sparkyRight.getEncoder().setPosition(0);
  }

}


