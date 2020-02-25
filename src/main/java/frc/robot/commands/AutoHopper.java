/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class AutoHopper extends CommandBase {
  /**
   * Creates a new AutoHopper.
   */
  Hopper hopper;
  double power;

  public AutoHopper(Hopper hopper, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopper = hopper;
    this.power = power;
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(hopper.getIntakeSensor()){
      hopper.setHopperPower(power);
    }

    if(hopper.getOuttakeSensor()){
      hopper.setHopperPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.setHopperPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!hopper.getIntakeSensor()){
      return true;
    }
    return false;
  }
}
