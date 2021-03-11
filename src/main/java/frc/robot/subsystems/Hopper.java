/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
  
  /**
   * Creates a new Hopper.
   */

   //change to mini neos
  private final CANSparkMax frontHopper;
  private final CANSparkMax backHopper;
  private final CANSparkMax indexerMaster;
  private final CANSparkMax indexerSlave;

  private final DigitalInput proximitySensorIntake;
  private final DigitalInput proximitySensorOuttake;

  
  private double ballCount = 0;

  public Hopper() {
    frontHopper = new CANSparkMax(Constants.RobotMap.kFrontHopper, MotorType.kBrushless);
    backHopper = new CANSparkMax(Constants.RobotMap.kBackHopper, MotorType.kBrushless);

    indexerMaster = new CANSparkMax(Constants.RobotMap.kIndexerMaster, MotorType.kBrushless);
    indexerSlave = new CANSparkMax(Constants.RobotMap.kIndexerSlave, MotorType.kBrushless);

    proximitySensorIntake = new DigitalInput(Constants.RobotMap.kHopperIntakeProximitySensor);
    proximitySensorOuttake = new DigitalInput(Constants.RobotMap.kHopperOuttakeProximitySensor);

    indexerMaster.setInverted(false);
    indexerSlave.follow(indexerMaster);

    backHopper.setInverted(true);
    frontHopper.setInverted(false);
    backHopper.follow(frontHopper);

    frontHopper.setOpenLoopRampRate(HopperConstants.kRampRate);

  }

  public void setHopperPower(double power){
    frontHopper.set(power);
  }

  public void setIndexer(double power){
    indexerMaster.set(power);
  }
  
  @Log 
  public boolean getIntakeSensor(){
    return !proximitySensorIntake.get();
  }

  @Log
  public boolean getOuttakeSensor(){
    return !proximitySensorOuttake.get();
  }

  public double clearCount(){
    ballCount = 0;
    return ballCount;
  }

  public double addBall(){
    ballCount ++;
    return ballCount;
  }

  public double getBallCount() {
    return ballCount;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
