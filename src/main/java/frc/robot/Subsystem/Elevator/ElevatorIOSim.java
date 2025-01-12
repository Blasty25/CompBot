// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {


    private final LinearSystem<N2, N1, N2> leftSim = 
        LinearSystemId.createElevatorSystem(DCMotor.getNEO(1), Units.lbsToKilograms(14), 1.3, 1.5);
    private final LinearSystem <N2, N1, N2> rightSim =
        LinearSystemId.createElevatorSystem(DCMotor.getNEO(1), Units.lbsToKilograms(14), 1.3, 1.5);

    DCMotorSim sparkyLeft = new DCMotorSim(leftSim, DCMotor.getNEO(1));
    DCMotorSim sparkyRight = new DCMotorSim(rightSim, DCMotor.getNEO(1));

    double leftAppliedVolts = 0.0;
    double rightAppliedVolts = 0.0;

      //From lines to 50 to 63 sim stuff have to move to a different class
//   private final LinearSystem<N2, N1, N2> elevator =
//      LinearSystemId.createElevatorSystem(DCMotor.getNEO(1), 3, 1.3, 1.5);


//   public ElevatorSim m_elevator = new ElevatorSim(
//       elevator,
//       DCMotor.getNEO(2),
//       Units.inchesToMeters(3),
//       Units.inchesToMeters(10),
//       false,
//       Units.inchesToMeters(0)
//       );

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        sparkyLeft.update(0.025);
        sparkyRight.update(0.025);

        inputs.leftAppliedVolts = sparkyLeft.getAngularVelocityRadPerSec();
        inputs.rightAppliedVolts = sparkyRight.getAngularVelocityRadPerSec();
    }

    @Override
    public void setManualSpeed(double volts) {
        leftAppliedVolts = MathUtil.clamp(volts, -12, 12);
        rightAppliedVolts = MathUtil.clamp(volts, -12, 12);
        Logger.recordOutput("Left Elevator Output", leftAppliedVolts);
        Logger.recordOutput("Right Elevator Output", rightAppliedVolts);
        sparkyLeft.setInputVoltage(leftAppliedVolts);
        sparkyRight.setInputVoltage(rightAppliedVolts);
    }

}
