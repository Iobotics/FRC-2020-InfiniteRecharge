/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utilities;

import frc.robot.Constants.DrivetrainConstants;

/**
 * Class for Utlity and Math function
 */
public final class Utils {
    /**
     * 
     * @param input
     * @return takes input number and returns its sign (e.g. -100 becomes -1, 69 becomes 1, etc.)
     */
     public static double absSign(double input){
        return (Math.abs(input) / input);
    }

    public static double degreesToRadians(double degrees){
        return (Math.PI * degrees) / 180;
    }

    public static double feetToMeters(double feet){
        return feet * 0.3048;
    }

    public static double ticksToInches(double ticks){
        return ((ticks / 2048) / DrivetrainConstants.kGearRatio) * Math.PI * DrivetrainConstants.kWheelDiameter;
    }

    public static double inchesToTicks(double inches){
        return (inches * DrivetrainConstants.kGearRatio)/(DrivetrainConstants.kWheelDiameter*Math.PI);
    }

    public static double ticksToMeters(double ticks){
        return feetToMeters(ticksToInches(ticks) / 12);
    }

}
