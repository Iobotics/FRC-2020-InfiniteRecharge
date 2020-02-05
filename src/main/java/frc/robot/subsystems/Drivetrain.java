/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

  private WPI_TalonSRX leftMaster;
  private WPI_TalonSRX rightMaster;
  private WPI_TalonSRX leftSlave;
  private WPI_TalonSRX rightSlave;

  private DifferentialDrive tankDrive;

  private AHRS gyro;

  private DifferentialDriveOdometry driveOdometry;
  private Pose2d robotPose;

  public Drivetrain() {
    leftMaster = new WPI_TalonSRX(RobotMap.kLeftMaster);
    rightMaster =  new WPI_TalonSRX(RobotMap.kRightMaster);
    leftSlave = new WPI_TalonSRX(RobotMap.kLeftSlave);
    rightSlave = new WPI_TalonSRX(RobotMap.kRightSlave);

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

  public double getAngle(){
    return gyro.getAngle();
  }

  public AHRS getGyro(){
    return gyro;
  }


  /**
   * Updates the bots Position and returns the value as a pose 2d 
   * @return Pose2D of bot, containing X, Y, and Rotation Values
   * */
  public Pose2d updateOdometry(){
    return driveOdometry.update(new Rotation2d(Utils.degreesToRadians(gyro.getAngle())), 
    leftMaster.getSelectedSensorPosition(), 
    rightMaster.getSelectedSensorPosition());
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
