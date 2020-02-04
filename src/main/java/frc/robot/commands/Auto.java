/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.Utilities.Utils;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Auto extends PIDCommand {
  /**
   * Creates a new Auto.
   */

   static PIDController PID;
   Timer time = new Timer();
   AHRS gyro;

  public Auto(AHRS gyro, double angle, double initialAngle, Drivetrain drive) {
    super(
        // The controller that the co + gyro.getAngle(mmand will use
        PID = new PIDController(0.0135 * 0.8, 0.00, (0.0135 * 1.4)/10),
        //new PIDController(0.0135 * 0.6, 1.2 * (0.0135 / 1.4), (0.0135 * 1.4 * 3)/40),

        // This should return the measurement
        gyro::getAngle,
        // This should return the setpoint (can also be a constant)
        () -> angle + initialAngle,
        // This uses the output
        output -> {
          drive.setTank(output + Utils.absSign(output) * 0.0, (-output + Utils.absSign(-output) * 0.0));
          //drive.setTank(output, output);
        });
        this.gyro = gyro;
        time.start();
        addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double gyroAngles [] = {0, 0};
    gyroAngles[1] = gyroAngles[0];
    gyroAngles[0] = gyro.getAngle();
    if (time.get() > 3 && (PID.getPositionError() < 1 || PID.getPositionError() > -1) && (gyroAngles[0] - gyroAngles[1] == 0)){
      time.stop();
      time.reset();
      return true;
    }else{
      return false;
    }
  }
}
