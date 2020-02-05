/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utilities;

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

}
