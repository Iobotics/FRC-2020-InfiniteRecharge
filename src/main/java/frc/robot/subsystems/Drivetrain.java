/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Collection;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;

public class Drivetrain extends SubsystemBase {

  private TalonFX leftMaster;
  private TalonFX rightMaster;
  private TalonFX leftSlave;
  private TalonFX rightSlave;

  private Orchestra orchestra;
  private Collection<TalonFX> instruments;

  public enum CHRP {
    MiiChannel("music/mii.chrp"), 
    StarWarsImperialMarch("music/star_wars_imperial.chrp"),
    RussianAnthem("music/russian_anthem");

    private String path = "";

    private CHRP(String path) {
      this.path = path;
    }
  }

  public Drivetrain() {
    leftMaster = new TalonFX(RobotMap.kLeftMaster);
    rightMaster =  new TalonFX(RobotMap.kRightMaster);
    leftSlave = new TalonFX(RobotMap.kLeftSlave);
    rightSlave = new TalonFX(RobotMap.kRightSlave);
    
    instruments.add(leftMaster);
    instruments.add(leftSlave);
    instruments.add(rightMaster);
    instruments.add(rightSlave);

    orchestra = new Orchestra(instruments);

    //Set Motor Polarities
    leftMaster.setInverted(false);
    leftSlave.setInverted(false);
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);

    //SetupSensor
    leftMaster.setSensorPhase(false);

    //Slave motors
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    //Config Slave Deadband
    leftSlave.configNeutralDeadband(0);
    rightSlave.configNeutralDeadband(0);

    //Config Ramp Rate
    leftMaster.configOpenloopRamp(0.50);
    rightMaster.configOpenloopRamp(0.50);

    //Config NeutralMode to brake
    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

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

  public class Music {
    public void play() {
      orchestra.play();
    }
    public void play(CHRP music) {
      orchestra.loadMusic(music.path);
      orchestra.play();
    }

    public void pause() {
      orchestra.pause();
    }

    public void stop() {
      orchestra.stop();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
