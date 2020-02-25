/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utilities;

import edu.wpi.first.wpilibj.controller.*;

/**
 * PIDController that automatically calculates PID based on Ziegler Nichols Tuning Rules.
 * In order to tune Increase the P-gain until stable oscillation is achieved. Oscillation should not Diverge nor Converge on 0
 * Use the P-gain found and the Period of the oscillation for the calculations
 */
public class ZiegNichPID extends PIDController {

    /**
     *
     * Enum to determin which combination of PID is being used
     *   
     * */ 
    public enum ControlType {
        P, PI, PD, PID;
    }
    
    /**
     * @param kU the calculated Ultimate P gain of the controller
     * @param type the combination of P I and D that will be used
     */
    private final double calcP(double kU, ControlType type){
        switch(type){

            case P:
                return 0.5 * kU;

            case PI:
                return 0.45 * kU;

            case PD:
                return 0.8 * kU;

            case PID:
                return 0.6 * kU;

            default: 
                return 0;
        }
    }

    /**
     * @param kU the calculated Ultimate P gain of the controller
     * @param tU the period of the 
     * @param type the combination of P I and D that will be used
     */

    private double calcI(double kU, double tU, ControlType type){
        switch(type){

            case P:
                return 0;

            case PI:
                return (0.54 * kU) / tU;

            case PD:
                return 0;

            case PID:
                return (1.2 * kU) / tU;
                
            default: 
                return 0;
        }
    }

    /**
     * @param kU the calculated Ultimate P gain of the controller
     * @param tU the period of the 
     * @param type the combination of P I and D that will be used
     */

    private double calcD(double kU, double tU, ControlType type){
        switch(type){

            case P:
                return 0;

            case PI:
                return 0;

            case PD:
                return (tU * kU) / 10;

            case PID:
                return (3 * tU * kU) / 40;
                
            default: 
                return 0;
        }
    }

    public ZiegNichPID(double kU, double tU, ControlType type) {

        super(0, 0, 0);

        this.setP(calcP(kU, type));
        this.setI(calcI(kU, tU, type));
        this.setD(calcD(kU, tU, type));
        // Pass in calculated values using Ziegler Nichols tuning rules
    }
}
