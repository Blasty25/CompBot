// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Elevator;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {


    private final LinearSystem<N2, N1, N2> rightSparky = 
        LinearSystemId.createElevatorSystem(DCMotor.getNEO(1), 1, 1.3, 1.5);

}
