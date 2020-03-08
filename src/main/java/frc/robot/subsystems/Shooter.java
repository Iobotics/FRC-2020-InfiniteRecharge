/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private TalonSRX leftShooter;
  private TalonSRX rightShooter;

  private TalonSRX articulatiungHood;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    leftShooter = new TalonSRX(RobotMap.kLeftShooter);
    rightShooter = new TalonSRX(RobotMap.kRightShooter);
    leftShooter.configFactoryDefault();
    rightShooter.configFactoryDefault();

    articulatiungHood = new TalonSRX(RobotMap.kArticulatingHood);
    articulatiungHood.configFactoryDefault();

    rightShooter.follow(leftShooter);
    rightShooter.setInverted(true);

    leftShooter.config_kF(0, ShooterConstants.kF);
    leftShooter.config_kP(0, ShooterConstants.kP);
    leftShooter.config_kI(0, ShooterConstants.kI);
    leftShooter.config_kD(0, ShooterConstants.kD);

    leftShooter.setNeutralMode(NeutralMode.Brake);
    articulatiungHood.setNeutralMode(NeutralMode.Brake);

    articulatiungHood.config_kP(0, HoodConstants.kP);
    articulatiungHood.config_kI(0, HoodConstants.kI);
    articulatiungHood.config_kD(0, HoodConstants.kD);
    articulatiungHood.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    articulatiungHood.setSensorPhase(false);
  }

  public void setPercent(double percent) {
    leftShooter.set(ControlMode.PercentOutput, percent);
  }

  public void setVelocity(double rpm) {
    //Convert Revolutions per Minute (RPM) to Cycles per 100 milliseconds (cpm) for closed loop controller
    double cpm = rpm * 600 / 8192; 
    leftShooter.set(ControlMode.Velocity, cpm); 
  }

  public void stopShooter() {
    leftShooter.set(ControlMode.PercentOutput, 0);
  }

  /**
   * 
   * @param percentFoward proportionall how far foward the hood is
   */
  public void setHood (double percentFoward) {
    articulatiungHood.set(ControlMode.Position, (percentFoward * (HoodConstants.hoodTop - HoodConstants.hoodBottom)) + HoodConstants.hoodBottom);
  }

  public void setManualHood (double speed) {
    articulatiungHood.set(ControlMode.PercentOutput, speed);
  } 

  @Override
  public void periodic() {
  }
}
