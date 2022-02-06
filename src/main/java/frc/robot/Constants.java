// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Color;

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
    }

    public static final class InternalConstants {
        public static final int motorPortBottom = 13;
        public static final int motorPortTop = 14;

        public static final int entranceIRPort = 0;
        public static final int exitIRPort = 1;

        public static final Color RED = new Color(0.561, 0.232, 0.114);
        public static final Color BLUE = new Color(0.143, 0.427, 0.429); 
        public static Color allianceColor = RED;
    }

    public static final class IntakeConstants {
        public static final int intakePort = 12;
        public static final int deploymentPort = 11;
    }

    public static final class ClimbConstants {
        public static final int sixMotorPort = 0;
        public static final int sixBrakePort = 1;

        public static final int tenMotorPort = 3;
        public static final int tenBrakePort = 2;

        public static final int tenLeftSolenoidPort = 18;
        public static final int tenRightSolenoidPort = 18;

        public static final int fifteenLeftPort = 10;
        public static final int fifteenRightPort = 19;
    }

    public static final class JetsonConstants {
        public static final int intakeCameraPort = 1181;
        public static final int turretCameraPort = 1182;
        public static final String jetsonAddress = "10.1.92.94";
    }
}
