// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

       private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(DCMotor.getNEO(2), Units.lbsToKilograms(30.0), Units.inchesToMeters(0.9175), 6.75),
        DCMotor.getNEO(2),
        Units.inchesToMeters(0),
        Units.inchesToMeters(44.0),
        true,
        Units.inchesToMeters(0)
    );

    private PIDController feedback = new PIDController(5.8256,0, 1.0701);
    private double feedforward = 0.0;
    
    @Override
    public void updateInputs(ElevatorInputs inputs) {
        double position = sim.getPositionMeters();

        double voltage = MathUtil.clamp(feedforward + feedback.calculate(position), -12.0, 12.0);
        sim.setInputVoltage(voltage);
        sim.update(0.02);

        inputs.elevatorPosition = position;
        inputs.encoderVelocity = sim.getVelocityMetersPerSecond();
        inputs.currents = new double[] {sim.getCurrentDrawAmps()};
    }

    @Override
    public void setManualSpeed(double output) {
        feedback.setSetpoint(output);
        sim.setInput(output);
    }

}
