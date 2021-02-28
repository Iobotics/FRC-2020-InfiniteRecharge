/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Utilities.Utils;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PrepShoot extends SequentialCommandGroup {
  /**
   * Creates a new ShootBalls.
   */
  public PrepShoot(double hoodAngle, double rpm, Shooter shooter, Lidar lidar){
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new InstantCommand(()-> shooter.setHood(Utils.findAngle(lidar)), shooter),
      new InstantCommand(()-> shooter.setVelocity(rpm), shooter)
    );
  }
}
