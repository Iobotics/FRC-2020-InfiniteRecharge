/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Collection;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;

public class Drivetrain extends SubsystemBase {

  private WPI_TalonFX leftMaster;
  private WPI_TalonFX rightMaster;
  private WPI_TalonFX leftSlave;
  private WPI_TalonFX rightSlave;

  private DifferentialDrive drive;

  public Drivetrain() {
    leftMaster = new WPI_TalonFX(RobotMap.kLeftMaster);
    rightMaster = new WPI_TalonFX(RobotMap.kRightMaster);
    leftSlave = new WPI_TalonFX(RobotMap.kLeftSlave);
    rightSlave = new WPI_TalonFX(RobotMap.kRightSlave);

    drive = new DifferentialDrive(leftMaster, rightMaster);
    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightSlave.configFactoryDefault();
    //Set Motor Polarities
    leftMaster.setInverted(false);
    leftSlave.setInverted(false);
    rightMaster.setInverted(false);
    rightSlave.setInverted(false);

    //SetupSensor
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftMaster.setSensorPhase(false);

    //Slave motors
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    //Config Slave Deadband
    leftSlave.configNeutralDeadband(0);
    rightSlave.configNeutralDeadband(0);

    //Config Ramp Rate
    leftMaster.configOpenloopRamp(1);
    rightMaster.configOpenloopRamp(1);

    //Config NeutralMode to brake
    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);

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

  public void setArcade(double speed, double rotation){
    drive.arcadeDrive(speed, rotation);
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
    //converting allowed error from inches to encoder units
    double allowedError = ((error * DrivetrainConstants.kGearRatio)/(DrivetrainConstants.kWheelDiameter * Math.PI) * 2048);
    if(Math.abs(leftMaster.getSelectedSensorPosition() - targetPos) <= allowedError && leftMaster.getSelectedSensorVelocity() == 0.0 && leftMaster.getActiveTrajectoryVelocity() < 3) {
      return true;
    } else{
      return false;
    }
  }

  public int getVelocity() {
    return leftMaster.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
