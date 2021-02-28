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
import frc.robot.Constants.RobotMap;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  private TalonSRX intake;
  private TalonSRX intakeSlave;

  public Intake() {
    intake = new TalonSRX(RobotMap.kIntake);
    intakeSlave = new TalonSRX(RobotMap.kIntakeSlave);
    intakeSlave.setInverted(true);
    intakeSlave.follow(intake);
  }

  public void setPercent(double demand) {
    intake.set(ControlMode.PercentOutput, demand);
    intakeSlave.set(ControlMode.PercentOutput, demand);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
