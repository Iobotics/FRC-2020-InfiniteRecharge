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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  private final TalonSRX frontHopper;
  private final TalonSRX backHopper;

  private final DigitalInput proximitySensorIntake;
  private final DigitalInput proximitySensorOuttake;

  public Hopper() {
    frontHopper = new TalonSRX(Constants.RobotMap.kFrontHopper);
    backHopper = new TalonSRX(Constants.RobotMap.kBackHopper);

    proximitySensorIntake = new DigitalInput(Constants.RobotMap.kHopperIntakeProximitySensor);
    proximitySensorOuttake = new DigitalInput(Constants.RobotMap.kHopperOuttakeProximitySensor);

    backHopper.setInverted(true);
    frontHopper.setInverted(false);
    backHopper.follow(frontHopper);
  }

  public void setHopperPower(double power){
    frontHopper.set(ControlMode.PercentOutput, power);
  }

  public boolean getIntakeSensor(){
    return proximitySensorIntake.get();
  }

  public boolean getOuttakeSensor(){
    return proximitySensorOuttake.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
