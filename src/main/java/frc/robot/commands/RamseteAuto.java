/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class RamseteAuto extends RamseteCommand {

  /**
   * Class that uses trajectory following to follow a set path
   * @param drivetrain the drivetrain subsystem to control the bot
   * @param trajectory the trajectory of the bot, can be made manually or loaded from external program. For this project they are stored in trajectories classs
   */

  public RamseteAuto(Drivetrain drivetrain, Trajectory trajectory){
    super(trajectory, drivetrain::getPose,
      new RamseteController(DrivetrainConstants.kRamseteB, DrivetrainConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DrivetrainConstants.ksVolts, DrivetrainConstants.kMaxVel, DrivetrainConstants.kMaxAcc),
      drivetrain.getKinematics(),
      drivetrain::getWheelSpeeds,
      new PIDController(DrivetrainConstants.kPRamseteVel, 0, 0),
      new PIDController(DrivetrainConstants.kPRamseteVel, 0, 0),
      drivetrain::setVoltage,
      drivetrain
    );
  } 
}
