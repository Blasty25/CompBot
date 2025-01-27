// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs{
        public double leftAppliedVolts = 0.0;
        public double rightAppliedVolts = 0.0;
        public double leftSparkTemp = 0.0;
        public double rightSparkTemp = 0.0;
        public double encoderVelocity = 0.0;
        public double elevatorPosition = 0.0;
        public double[] currents = new double[] {};
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void setElevator(double setPoint, double voltage) {}

    public default void setManualSpeed(double output) {}

    public default void setPosition(double position, double ffVoltage) {}
}
