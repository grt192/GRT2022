// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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
        public static final int fLeftMotorPort = 4;
        public static final int mLeftMotorPort = 5;
        public static final int bLeftMotorPort = 6;

        public static final int fRightMotorPort = 9;
        public static final int mRightMotorPort = 8;
        public static final int bRightMotorPort = 7;
    }

    public static final class TurretConstants {
        public static final int turntablePort = 17;
        public static final int hoodPort = 16;
        public static final int flywheelPort = 15;

        public static final int lLimitSwitchPort = 4;
        public static final int rLimitSwitchPort = 5;
    }

    public static final class InternalConstants {
        public static final double RAISE_POW = 0.8;
        
        public static final int motorPortBottom = 13;
        public static final int motorPortTop = 14;

        public static final int entranceIRPort = 0;
        public static final int stagingIRPort = 1;
    }

    public static final class IntakeConstants {
        public static final int intakePort = 12;
        public static final int deploymentPort = 11;
        public static final int tLimitSwitchPort = 3;
        public static final int bLimitSwitchPort = 4;
    }

    public static final class ClimbConstants {
        public static final int sixMotorPort = 3;
        public static final int sixBrakePort = 2;

        public static final int tenMotorPort = 1;
        public static final int tenBrakePort = 0;

        public static final int tenLeftSolenoidPort = 18;
        public static final int tenRightSolenoidPort = 18;

        public static final int fifteenLeftPort = 10;
        public static final int fifteenRightPort = 19;
    }

    public static final class JetsonConstants {
        public static final int turretCameraPort = 1181;
        public static final int intakeCameraPort = 1182;
        public static final int jetsonPort = 5800;
        public static final String jetsonIP = "10.1.92.12";
    }

    public static final class ShuffleboardConstants {
        public static final double UPDATE_TIME = 0.5; // seconds between each update
    }

    /**
     * Ball coordinates, based off of Kepler's work on onshape:
     * https://pausd.onshape.com/documents/8003f7997b588f082d72c477/w/14e5661479750659f282841e/e/4af5ee78827743d1d23aca5f
     * All coordinates assume the field coordinate system (positive x is right, positive y is up, theta ccw from
     * x axis) with the origin at the hub.
     */
    public static final class BallCoordinates {
        // Tarmac ball translations. These do not include angle because we may choose to approach these balls at
        // different angles based on our starting position. Declared in clockwise order around the tarmac from 
        // top right to top left ball.
        public static final Translation2d 
            RIGHT_TOP_BLUE = new Translation2d(Units.inchesToMeters(-33.763), Units.inchesToMeters(149.233)),
            RIGHT_TOP_RED = new Translation2d(Units.inchesToMeters(25.907), Units.inchesToMeters(150.795)),
            RIGHT_MID_RED = new Translation2d(Units.inchesToMeters(124.952), Units.inchesToMeters(88.302)),
            RIGHT_MID_BLUE = new Translation2d(Units.inchesToMeters(149.230), Units.inchesToMeters(33.772)),
            RIGHT_BOTTOM_RED = new Translation2d(Units.inchesToMeters(129.397), Units.inchesToMeters(-81.649)),
            RIGHT_BOTTOM_BLUE = new Translation2d(Units.inchesToMeters(88.309), Units.inchesToMeters(-124.947)),
            LEFT_BOTTOM_RED = new Translation2d(Units.inchesToMeters(33.763), Units.inchesToMeters(-149.233)),
            LEFT_BOTTOM_BLUE = new Translation2d(Units.inchesToMeters(-25.907), Units.inchesToMeters(-150.795)),
            LEFT_MID_BLUE = new Translation2d(Units.inchesToMeters(-124.952), Units.inchesToMeters(-88.302)),
            LEFT_MID_RED = new Translation2d(Units.inchesToMeters(-149.230), Units.inchesToMeters(-33.772)),
            LEFT_TOP_BLUE = new Translation2d(Units.inchesToMeters(-129.397), Units.inchesToMeters(81.649)),
            LEFT_TOP_RED = new Translation2d(Units.inchesToMeters(-88.309), Units.inchesToMeters(124.947));

        // Terminal ball translations. These should generally be approached at 45 and 225 degrees, respectively.
        public static final Translation2d
            TERMINAL_RED = new Translation2d(Units.inchesToMeters(284.210), Units.inchesToMeters(120.435)),
            TERMINAL_BLUE = new Translation2d(Units.inchesToMeters(-284.210), Units.inchesToMeters(-120.435));
    }
}
