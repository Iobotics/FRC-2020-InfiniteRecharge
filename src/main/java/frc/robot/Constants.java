/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class RobotMap{
        //Drivetrain
        public static final int kLeftMaster = 0;
        public static final int kRightMaster = 2;
        public static final int kLeftSlave = 1  ;
        public static final int kRightSlave = 3;

        //Intake
        public static final int kIntake = 4;
        public static final int kIntakeSlave = 5;

        //Hopper
        public static final int kFrontHopper = 6;
        public static final int kBackHopper = 7;
        public static final int kIndexerMaster = 13;
        public static final int kIndexerSlave = 14;
        public static final int kHopperIntakeProximitySensor = 0;
        public static final int kHopperOuttakeProximitySensor = 1;
        
        //Articulating Hood
        public static final int kLeftShooter = 8;
        public static final int kRightShooter = 9;
        public static final int kArticulatingHood = 10;

        //Lift
        public static final int kLiftMaster = 12;
        public static final int kLiftSlave = 11;

        //LED Strip
        public static final int kArduino = 21;
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
        public static final double kWheelDiameter = 8.00;        //in inches

        //PID Values
        public static final float kP = 0.05f;
        public static final float kI = 0f;
        public static final float kD = 0f;
        
        //Feed Forward
        public static final float kF = 0.0457f;
    }

    public static final class ShooterConstants {
        //PID Values
        public static final float kP = 0f;
        public static final float kI = 0f;
        public static final float kD = 0f;
        //Feed Forward
        public static final float kF = 0f;
    }

    public static final class HoodConstants {
        //PID Values
        public static final float kP = 4f;
        public static final float kI = 0f;
        public static final float kD = 40f;

        //Encoder Values for the top and bottom position of the articulating hood
        public static final int hoodBottom = 680;
        public static final int hoodTop = 420;
    }

    public static final class liftConstants {
        
        //Velocity PID values to stop the lift
        public static final float kP = 0f;
        public static final float kI = 0f;
        public static final float kD = 0f;
        public static final float kF = 0f;
    }

    public static final class LEDConstants {
        //Count starting from 0
        public static final int kFirst = 0;
        public static final int kLast = 178; 
    }

    public static final class HopperConstants {
        public static final double kRampRate = 0.5;
    }
}
