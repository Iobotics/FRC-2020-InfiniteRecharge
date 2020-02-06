/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Utilities.Utils;

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
    public static final class RobotMap{
        //Drivetrain
        public static final int kLeftMaster = 1;
        public static final int kRightMaster = 4;
        public static final int kLeftSlave = 2;
        public static final int kRightSlave = 3;

        public static final int kControlPanelSpinner = 6;

        public static final int kIntake = 9;
        
        public static final int kLeftShooter = 7;
        public static final int kRightShooter = 10;
        public static final int kArticulatingHood = 5;
    }

    public static final class OIConstants{
        public static final int kJoystick1 = 0;
        public static final int kJoystick2 = 1;
        public static final int kXboxController = 2;

        public static final int kRunIntake = 1;
        public static final int kSpinWheel = 2;

        //Xbox controller
        public static final int kRunShooter = 6;
    }

    public static final class DrivetrainConstants {
        public static final double kGearRatio = 10.71;
        //in inches
        public static final double kWheelDiameter = 8.00;

        //PID Values
        public static final float kP = 0.05f;
        public static final float kI = 0f;
        public static final float kD = 0f;
        //Feed Forward
        public static final float kF = 0.0457f;


        /*
        *Kinematics Section
        */

        public static final double kTrackWidth = Utils.feetToMeters(27.5/12);

        //Feedfoward
        //TODO: Tune values
        public static final double ksVolts = 0;
        public static final double kvVoltSecondsPerMeter = 0;
        public static final double kaVoltSecondsSquaredPerMeter = 0;

        //Max Velocity and Acceleration M/S
        //TODO: find better values
        public static final double kMaxVel = 3;
        public static final double kMaxAcc = 3;

        //Ramsete Variables
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    
    }


    public static final class ShooterConstants {
        //PID Values
        public static final float kP = 0f;
        public static final float kI = 0f;
        public static final float kD = 0f;
        //Feed Forward
        public static final float kF = 0f;
    }
}
