/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

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
    liftMaster.setInverted(false);
    
    //Configure Limti Switches for lift, bottom Switch is on the slave and top switch is on the master
    liftMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    liftMaster.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, RobotMap.kLiftSlave);
    liftMaster.overrideLimitSwitchesEnable(false);
  }

  public void setLift(double power){
    liftMaster.set(ControlMode.PercentOutput, power);
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
