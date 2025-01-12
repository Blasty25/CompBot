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
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorIOSparkMax implements ElevatorIO {
    
    ElevatorInputsAutoLogged inputs;

    private final ProfiledPIDController m_controller = new ProfiledPIDController(
    ElevatorConstants.kP,
    ElevatorConstants.kI,
    ElevatorConstants.kP,
    new TrapezoidProfile.Constraints(2.45, 2.45));

    private AbsoluteEncoder encoder;
    private PIDController pid = new PIDController(0.5, 0, 0);

    private SparkMax sparkyLeft;
    private SparkMax sparkyRight;

    private SparkMaxConfig sparkyLeftConfig = new SparkMaxConfig();
    private SparkMaxConfig sparkyRightConfig = new SparkMaxConfig();

    public ElevatorIOSparkMax(){
    inputs = new ElevatorInputsAutoLogged();
    sparkyLeft = new SparkMax(ElevatorConstants.sparkyLeft, MotorType.kBrushless);
    sparkyRight = new SparkMax(ElevatorConstants.sparkyRight, MotorType.kBrushless);
    encoder = sparkyLeft.getAbsoluteEncoder();


    //Configuring the Encoder on the Spark Max is not Avliable until complete documentation
    //https://docs.revrobotics.com/brushless/revlib/revlib-overview/migrating-to-revlib-2025
    //Using a external encoder for now
    sparkyLeftConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast);
    sparkyLeftConfig.absoluteEncoder
      .positionConversionFactor(6)
      .velocityConversionFactor(6.0/60.0);
    sparkyLeftConfig.closedLoop
      .pid(0.5, 0, 0)
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    sparkyRightConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast);
    sparkyRightConfig.absoluteEncoder
        .positionConversionFactor(6)
        .velocityConversionFactor(6.0/60.0);
    sparkyRightConfig.closedLoop
        .pid(0.50, 0, 0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

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
    sparkyLeft.set((m_controller.calculate(setPoint / encoder.getPosition())) * ElevatorConstants.maxElevatorSpeed);
    sparkyRight.set(m_controller.calculate(setPoint / encoder.getPosition()) * ElevatorConstants.maxElevatorSpeed);
    // sparkyLeft.getClosedLoopController().setReference(setPoint, ControlType.kPosition);
    // sparkyRight.getClosedLoopController().setReference(setPoint, ControlType.kPosition);
    Logger.recordOutput("Encoder", encoder.getPosition());
    

    Logger.recordOutput("Sparky Left Output", setPoint);
    Logger.recordOutput("Sparky Right Output", setPoint);

  }

    @Override
  public void setManualSpeed(double volts) {
    sparkyLeft.set(volts);
    Logger.recordOutput("left Volts", volts);
    Logger.recordOutput("right Volts", volts);
  }

}


