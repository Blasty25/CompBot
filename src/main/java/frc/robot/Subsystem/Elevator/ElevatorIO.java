package frc.robot.Subsystem.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double position = 0.0;
        public double velocity = 0.0;

        public int currentLevel = 0;
        public int setLevel = 0;

        public double[] voltages = new double[] {};
        public double[] currents = new double[] {};
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setPosition(double position, double ffVoltage) {}

    public default void reset() {}
}
