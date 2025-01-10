// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public class ElevatorConstants{
        public static final int sparkyLeft = 1;
        public static final int sparkyRight = 2;
    }
      public final class DriveConstants {
          public static final int frontLeftDrive = 1;
          public static final int frontLeftTurn = 2;
          public static final int frontRightDrive = 3;
          public static final int frontRightTurn = 4;
          public static final int backLeftDrive = 5;
          public static final int backLeftTurn = 6;
          public static final int backRightDrive = 7;
          public static final int backRightTurn = 8;
      
          public static final int gyroID = 20;
      }

        public final class MapleSimSwerve {
          public static final double driveGearRatio = 2.5;
          public static final double turnGearRatio = 1.7;
          public static final double wheelDiameter = Units.metersToInches(0.0889);
        }

      public static final Mode currentMode = Mode.SIM;

          public static enum Mode {
            /** Running on a real robot. */
            REAL,
        
            /** Running a physics simulator. */
            SIM,
        
            /** Replaying from a log file. */
            REPLAY
          }
        
}