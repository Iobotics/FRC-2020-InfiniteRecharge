/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LEDStrip.LEDColor;
import frc.robot.subsystems.LEDStrip.LEDSector;

public class LEDController extends CommandBase {
  /**
   * Creates a new LEDController.
   */
  LEDStrip LEDs;
  Hopper hopper;
  Shooter shooter;
  public LEDController(LEDStrip LEDs, Hopper hopper, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(LEDs);
    this.LEDs = LEDs;
    this.hopper = hopper;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LEDs.setColorAlliance(DriverStation.getInstance().getAlliance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getActive()) {
      LEDs.setColor(LEDColor.GREEN, LEDSector.HOPPER_SIDES);
    } else if (hopper.getOuttakeSensor()) {
      LEDs.setColor(LEDColor.ORANGE, LEDSector.HOPPER_SIDES);
    } else {
      LEDs.setColorAlliance(DriverStation.getInstance().getAlliance());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDs.setColor(LEDColor.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.getInstance().getMatchTime() <= 0;
  }
}
