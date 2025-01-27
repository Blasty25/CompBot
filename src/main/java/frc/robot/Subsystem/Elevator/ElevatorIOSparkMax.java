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
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorIOSparkMax implements ElevatorIO {

    ElevatorInputsAutoLogged inputs;

    public SparkMax sparkyLeft;
    public SparkMax sparkyRight;
    private SparkAbsoluteEncoder encoder;

    private final SparkClosedLoopController feedbackLeft;

    public ElevatorIOSparkMax() {

        inputs = new ElevatorInputsAutoLogged();
        sparkyLeft = new SparkMax(30, MotorType.kBrushless);
        sparkyRight = new SparkMax(31, MotorType.kBrushless);

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        leftConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kCoast);
        leftConfig.encoder
                .positionConversionFactor(ElevatorConstants.positionConversionFactor)
                .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);
        leftConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAnalogSensor)
                .positionWrappingInputRange(0, 10)
                .p(ElevatorConstants.kP)
                .i(ElevatorConstants.kI)
                .d(ElevatorConstants.kD);

        rightConfig
                .smartCurrentLimit(40)
                .inverted(true)
                .idleMode(IdleMode.kCoast);
        rightConfig.encoder
                .positionConversionFactor(ElevatorConstants.positionConversionFactor)
                .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);
        rightConfig.closedLoop
                .p(ElevatorConstants.kP)
                .i(ElevatorConstants.kI)
                .d(ElevatorConstants.kD);

        sparkyLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sparkyRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = sparkyLeft.getAbsoluteEncoder();
        feedbackLeft = sparkyLeft.getClosedLoopController();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {

        inputs.elevatorPosition = sparkyLeft.getEncoder().getPosition();
        inputs.encoderVelocity = sparkyLeft.getEncoder().getVelocity();

        inputs.leftAppliedVolts = sparkyLeft.getAppliedOutput();
        inputs.rightAppliedVolts = sparkyRight.getAppliedOutput();

        inputs.leftSparkTemp = sparkyLeft.getMotorTemperature();
        inputs.rightSparkTemp = sparkyRight.getMotorTemperature();
    }

    @Override
    public void setManualSpeed(double output) {

    sparkyLeft.getClosedLoopController().setReference(output, ControlType.kPosition);
    sparkyRight.getClosedLoopController().setReference(output, ControlType.kPosition);

    }
}






                



/*
 
if (inputs.elevatorPosition >= 0.9) {
    sparkyLeft.getClosedLoopController().setReference((-Math.PI * ElevatorConstants.maxRotations),
            ControlType.kPosition);
    sparkyRight.getClosedLoopController().setReference((-Math.PI * ElevatorConstants.maxRotations),
            ControlType.kPosition);
}
if(inputs.elevatorPosition <= 0.2){
    sparkyLeft.getClosedLoopController().setReference((Math.PI * ElevatorConstants.maxRotations),
            ControlType.kPosition);
    sparkyRight.getClosedLoopController().setReference((Math.PI * ElevatorConstants.maxRotations),
            ControlType.kPosition);
}
 */
