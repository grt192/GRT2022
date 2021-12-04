// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class TankConstants {
        public static final int fLeftMotorPort = 15;
        public static final int fRightMotorPort = 1;
        public static final int bLeftMotorPort = 14;
        public static final int bRightMotorPort = 0;
    }

    public static final class ElevatorConstants {
        public static final int mainMotorPort = 5;
        public static final int followMotorPort = 6;

        public static final boolean elevatorUpIsPositive = true;

    }

    public static final class ClawConstants {
        public static final int motor1Port = 3;
        public static final int motor2Port = 12;

        public static final int pfftPCMPort = 0;

        public static final int clawOpenPosition = 10;
        public static final double clawMotorSpeed = 0.5;
    }
}
