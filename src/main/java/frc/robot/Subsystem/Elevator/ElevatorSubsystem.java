// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase implements ElevatorIO {
  /** Creates a new ElevatorSubsystem. */
  private final ProfiledPIDController m_controller = new ProfiledPIDController(
      ElevatorConstants.kP,
      ElevatorConstants.kI,
      ElevatorConstants.kP,
      new TrapezoidProfile.Constraints(2.45, 2.45));

  private SparkMax sparkyLeft;
  private SparkMax sparkyRight;

  private final Encoder m_Encoder = new Encoder(9, 10);

  private SparkMaxConfig sparkyLeftConfig = new SparkMaxConfig();
  private SparkMaxConfig sparkyRightConfig = new SparkMaxConfig();

  ElevatorInputsAutoLogged inputs;

  //From lines to 50 to 63 sim stuff have to move to a different class
  private final LinearSystem<N2, N1, N2> elevator =
     LinearSystemId.createElevatorSystem(DCMotor.getNEO(1), 3, 1.3, 1.5);


  public ElevatorSim m_elevator = new ElevatorSim(
      elevator,
      DCMotor.getNEO(2),
      Units.inchesToMeters(3),
      Units.inchesToMeters(10),
      false,
      Units.inchesToMeters(0)
      );

      //end of sim stuff

  public ElevatorSubsystem() {
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

    double pidOutput = m_controller.calculate(m_Encoder.getDistance());
    sparkyLeft.setVoltage(pidOutput);
    sparkyRight.setVoltage(pidOutput);
  }

  @Override
  public void setManualSpeed(double volts) {
    sparkyLeft.set(volts);
    sparkyRight.set(volts);
  }

  public Command setDutyCycle(DoubleSupplier volts) {
    return this.run(() -> {
      setManualSpeed(volts.getAsDouble());
    });

  }

  @Override
  public void periodic() {
    Logger.processInputs("Elevator", inputs);

    SmartDashboard.putData("Elevator 1 volt", setDutyCycle(() -> 1));
    SmartDashboard.putData("Elevator -1 volt", setDutyCycle(() -> -1));
  }
}
