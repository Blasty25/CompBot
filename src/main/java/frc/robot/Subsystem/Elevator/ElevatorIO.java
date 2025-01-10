// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs{
        public double appliedVolts = 0.0;
        public double leftSparkSet = 0.0;
        public double rightSparkSet = 0.0;
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void setElevator(double volts) {}

}
