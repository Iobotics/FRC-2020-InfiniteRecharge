/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap;

public class Drivetrain extends SubsystemBase {

  private TalonFX leftMaster;
  private TalonFX rightMaster;
  private TalonFX leftSlave;
  private TalonFX rightSlave;

  public Drivetrain() {
    leftMaster = new TalonFX(RobotMap.kLeftMaster);
    rightMaster =  new TalonFX(RobotMap.kRightMaster);
    leftSlave = new TalonFX(RobotMap.kLeftSlave);
    rightSlave = new TalonFX(RobotMap.kRightSlave);
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  }
  
  public void setTank(double leftPower, double rightPower){
    if (leftPower <= 0.3 && leftPower >= -0.3) {
      leftPower = 0;
    }
    if (rightPower <= 0.3 && rightPower >= -0.3) {
      rightPower = 0;
    }
    leftMaster.set(ControlMode.PercentOutput, Math.pow(leftPower, 3));
    rightMaster.set(ControlMode.PercentOutput, Math.pow(rightPower, 3));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
