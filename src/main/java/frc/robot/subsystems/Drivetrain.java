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
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    //Configure PIDF values for Auto drive, the Left Master is the master controller for PID
    leftMaster.config_kP(0, DrivetrainConstants.kP);
    leftMaster.config_kI(0, DrivetrainConstants.kI);
    leftMaster.config_kD(0, DrivetrainConstants.kD);
    leftMaster.config_kF(0, DrivetrainConstants.kF);
  }

  /**
   * Reconfigures the motors to the drive settings
   */
  public void config () {
    rightMaster.configFactoryDefault();
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  } 

  public void stop() {
    leftMaster.set(ControlMode.PercentOutput, 0);
    rightMaster.set(ControlMode.PercentOutput, 0);
  }
  
  public void setTank(double leftPower, double rightPower){
    leftMaster.set(ControlMode.PercentOutput, leftPower);
    rightMaster.set(ControlMode.PercentOutput, rightPower);
  }

  /**
   * Moves to the given amount of inches using motion magic
   * @param distance distance to move (inches)
   * @param speed cruising speed of motor in inches per second
   */
  public void motionMagic (double distance, double speed) {
    double rotations = (distance * DrivetrainConstants.kGearRatio)/(DrivetrainConstants.kWheelDiameter*Math.PI);
    double targetPos = rotations*2048;
    //Convert target speed from inches / second to encoder units / 100 ms
    double targetSpeed = (speed *DrivetrainConstants.kGearRatio * 2048 * 10) / (DrivetrainConstants.kWheelDiameter * Math.PI);

    rightSlave.follow(leftMaster);
    rightMaster.follow(leftMaster);
    leftMaster.configMotionCruiseVelocity((int)targetSpeed);
    leftMaster.configMotionAcceleration((int)targetSpeed);
    leftMaster.setSelectedSensorPosition(0);
    leftMaster.set(ControlMode.MotionMagic, targetPos);
  }

  //Are we there yet
  public boolean isTargetAchieved (double distance, double error) {
    double rotations = (distance * DrivetrainConstants.kGearRatio)/(DrivetrainConstants.kWheelDiameter*Math.PI);
    double targetPos = rotations*2048;
    double allowedError = (error * DrivetrainConstants.kGearRatio * 2048);
    if(Math.abs(leftMaster.getSelectedSensorPosition() - targetPos) <=  3){
      return true;
    } else{
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
