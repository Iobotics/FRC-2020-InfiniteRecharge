/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;

public class Drivetrain extends SubsystemBase {

  private TalonSRX leftMaster;
  private TalonSRX rightMaster;
  private TalonSRX leftSlave;
  private TalonSRX rightSlave;

  public Drivetrain() {
    leftMaster = new TalonSRX(RobotMap.kLeftMaster);
    rightMaster =  new TalonSRX(RobotMap.kRightMaster);
    leftSlave = new TalonSRX(RobotMap.kLeftSlave);
    rightSlave = new TalonSRX(RobotMap.kRightSlave);
    rightMaster.setInverted(false);
    rightSlave.setInverted(false);
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.config_kP(0, DrivetrainConstants.kP);
    leftMaster.config_kI(0, DrivetrainConstants.kI);
    leftMaster.config_kD(0, DrivetrainConstants.kD);
    leftMaster.config_kF(0, DrivetrainConstants.kF);
  }

  public void config () {
    rightMaster.configFactoryDefault();
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  } 

  public void stop() {
    leftMaster.set(ControlMode.PercentOutput, )
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

  public void motionMagic (double distance, double speed) {

    double rotations = distance/(DrivetrainConstants.kGearRatio*DrivetrainConstants.kWheelDiameter*Math.PI);
    double targetPos = rotations*speed*4096;

    rightSlave.follow(leftMaster);
    rightMaster.follow(leftMaster);
    leftMaster.setSelectedSensorPosition(0);
    leftMaster.set(ControlMode.MotionMagic, targetPos);
  }

  //Are we there yet
  public boolean isTargetAchieved () {
    return leftMaster.isMotionProfileFinished();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
