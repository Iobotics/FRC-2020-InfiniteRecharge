/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Utilities.Utils;

public class Drivetrain extends SubsystemBase {

  private WPI_TalonFX leftMaster;
  private WPI_TalonFX rightMaster;
  private WPI_TalonFX leftSlave;
  private WPI_TalonFX rightSlave;

  private DifferentialDrive tankDrive;

  private AHRS gyro;

  private DifferentialDriveOdometry driveOdometry;
  private Pose2d robotPose;

  public Drivetrain() {
    leftMaster = new WPI_TalonFX(RobotMap.kLeftMaster);
    rightMaster =  new WPI_TalonFX(RobotMap.kRightMaster);
    leftSlave = new WPI_TalonFX(RobotMap.kLeftSlave);
    rightSlave = new WPI_TalonFX(RobotMap.kRightSlave);

    //Clear MotorValues
    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightSlave.configFactoryDefault();
    
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);

    //Create Differential Drivesudo
    tankDrive = new DifferentialDrive(rightMaster, rightMaster);
    tankDrive.setRightSideInverted(false);

    //Create Gyro Sensor
    gyro = new AHRS();

    //Create Drive Odometry starting 
    driveOdometry = new DifferentialDriveOdometry(new Rotation2d(Utils.degreesToRadians(gyro.getAngle())));

    //Set Motor Polarities
    leftMaster.setInverted(false);
    leftSlave.setInverted(false);
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);

    //SetupSensor
    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(false);

    //Slave motors
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    //Config Slave Deadband
    leftSlave.configNeutralDeadband(0);
    rightSlave.configNeutralDeadband(0);

    //Config Ramp Rate
    leftMaster.configOpenloopRamp(0.51);
    rightMaster.configOpenloopRamp(0.52);

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

  /**
   * Sets motor power based on voltage for trajectory control
   * @param leftVolt
   * @param rightVolt
   */
  public void setVoltage(double leftVolt, double rightVolt){
    leftMaster.setVoltage(leftVolt);
    rightMaster.setVoltage(leftVolt);
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

  /**
   * 
   * @return returns the angle as given by the gyro
   */
  public double getAngle(){
    return gyro.getAngle();
  }

  /**
   * 
   * @return returns the value of rotation from 0 - 360
   */
  public double getHeading(){
    return Math.IEEEremainder(getAngle(), 360);
  }

  public AHRS getGyro(){
    return gyro;
  }


  /**
   * Updates the bots Position and returns the value as a pose 2d 
   * @return Pose2D of bot, containing X, Y, and Rotation Values
   * */
  public Pose2d updateOdometry(){
    return driveOdometry.update(new Rotation2d(Utils.degreesToRadians(getHeading())), 
    Utils.ticksToMeters((double) leftMaster.getSelectedSensorPosition()), 
    Utils.ticksToMeters(rightMaster.getSelectedSensorPosition()));
  }

  public void resetOdometry(){
    driveOdometry.resetPosition(new Pose2d() ,new Rotation2d(Utils.degreesToRadians(getHeading())));
  }

  @Override
  public void periodic() {
    //Update Robot Position and Print Values to the Dashboard
    robotPose = updateOdometry();
    SmartDashboard.putNumber("Bot X", robotPose.getTranslation().getX());
    SmartDashboard.putNumber("Bot Y", robotPose.getTranslation().getX());
    SmartDashboard.putNumber("Bot Rotation", robotPose.getRotation().getDegrees());
  }
}
