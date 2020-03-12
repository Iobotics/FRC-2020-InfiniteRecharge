/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.liftConstants;

public class Lift extends SubsystemBase {
  /**
   * Creates a new Lift.
   */

  private final TalonSRX liftMaster;
  private final TalonSRX liftSlave;


  public Lift() {
    liftMaster = new TalonSRX(RobotMap.kLiftMaster);
    liftSlave = new TalonSRX(RobotMap.kLiftSlave);
    liftSlave.follow(liftMaster);
    liftMaster.setInverted(true);
    liftSlave.setInverted(false);
    liftMaster.setNeutralMode(NeutralMode.Brake);
    liftSlave.setNeutralMode(NeutralMode.Brake);
    
    //Configure Limti Switches for lift, bottom Switch is on the slave and top switch is on the master
    liftMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    liftMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    liftSlave.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, RobotMap.kLiftMaster);
    liftSlave.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, RobotMap.kLiftMaster);
    liftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    liftMaster.config_kP(0, liftConstants.kP);
    liftMaster.config_kI(0, liftConstants.kI);
    liftMaster.config_kD(0, liftConstants.kD);
    liftMaster.config_kF(0, liftConstants.kF);

  }

  public void setLift(double power){
    liftMaster.set(ControlMode.PercentOutput, power);
  }

  public void stopLift(){
    liftMaster.set(ControlMode.PercentOutput, 0);
    //liftMaster.set(ControlMode.Velocity, 0);
  }
  
  public boolean getForwardLimit(){
    return liftMaster.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getRevLimit(){
    return liftMaster.getSensorCollection().isRevLimitSwitchClosed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
