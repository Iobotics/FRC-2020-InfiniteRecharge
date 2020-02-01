/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Utils;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Auto extends PIDCommand {
  /**
   * Creates a new Auto.
   */

  public Auto(AHRS gyro, double angle, Drivetrain drive) {
    super(
        // The controller that the command will use
        //P was 0.012
        new PIDController(0.006, 0.006, 0.001),
        // This should return the measurement
        gyro::getAngle,
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          drive.setTank(-output + Utils.absSign(-output) * 0.0, (output + Utils.absSign(output) * 0.0));
          //drive.setTank(output, output);
        });
        addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
