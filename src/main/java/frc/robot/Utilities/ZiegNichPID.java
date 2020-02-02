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
 */
public class ZiegNichPID extends PIDController {

    public enum ControlType {
        P, PI, PD, PID;
    }

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
