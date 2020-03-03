/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.RamseteAuto;
import frc.robot.commands.ShootBalls;
import frc.robot.commands.AutoHopper;
import frc.robot.commands.PrepShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OIConstants;
import frc.robot.Utilities.Trajectories;
import frc.robot.Utilities.Utils;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.commands.Auto;
import frc.robot.commands.AutoAlign;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  
  private final Drivetrain drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight();
  private final Hopper hopper = new Hopper();
  private final LEDStrip ledStrip = new LEDStrip();
  private final Intake intake = new Intake();
  private final Shooter shooter =  new Shooter();
  private final Lidar lidar =  new Lidar();
  private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
  private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);

  private final XboxController xboxController = new XboxController(OIConstants.kXboxController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //Default Drivetrain command is tank drive
    drivetrain.setDefaultCommand(new RunCommand(
        () -> drivetrain.setTank(Math.pow(-joystick1.getY(), 3), Math.pow(-joystick2.getY(), 3)), drivetrain));
    
    ledStrip.setDefaultCommand(new RunCommand(
      () -> ledStrip.setColorAlliance(DriverStation.getInstance().getAlliance()), ledStrip
    ));

    //Default Limelight command 
    limelight.setDefaultCommand(new RunCommand(() -> limelight.printValues(), limelight));
    hopper.setDefaultCommand(new AutoHopper(hopper, 0.5));
    SmartDashboard.putNumber("Auto Number", 0);


    //Default Limelight command 
    limelight.setDefaultCommand(new RunCommand(() -> limelight.printValues(), limelight));
    hopper.setDefaultCommand(new AutoHopper(hopper, 0.5));
    SmartDashboard.putNumber("Auto Number", 0);
  }

  public double getGyro() {
    return drivetrain.getAngle();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick1, 2).whenPressed(new AutoAlign(limelight, drivetrain));
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /**
     * Switch for 4 different auto cases
     * Shooting from center and picking up balls in the shield
     * shooting and getting the trench balls
     * moving from wall and then shooting
     * and shooting from an unknown position
     */
    switch(SmartDashboard.getString("Auto Number", "")){
      //Picking up balls in the shield generator
      case "Shield":
        return new SequentialCommandGroup(
          new PrepShoot(Utils.findAngle(lidar), 20000, shooter),
          new ShootBalls(),
          new RamseteAuto(drivetrain, new Trajectories(drivetrain.getConfig()).getPath("AutoShield.path").relativeTo(drivetrain.getPose())),
          new ParallelCommandGroup(
            new AutoDrive(drivetrain, 84.75),
            new InstantCommand(()-> intake.setPercent(1), intake)
          ).andThen(new InstantCommand(()->intake.setPercent(0), intake)),
          new RamseteAuto(drivetrain, new Trajectories(drivetrain.getConfig()).getPath("AutoShieldReturn.path").relativeTo(drivetrain.getPose())),
          new PrepShoot(Utils.findAngle(lidar), 20000, shooter),
          new ShootBalls()
        );
      //Starting by the wall
      case "Wall":
        return new SequentialCommandGroup(
          new RamseteAuto(drivetrain, new Trajectories(drivetrain.getConfig()).getPath("AutoWall.path").relativeTo(drivetrain.getPose())),
          new PrepShoot(Utils.findAngle(lidar), 20000, shooter),
          new ShootBalls(),
          new RamseteAuto(drivetrain, new Trajectories(drivetrain.getConfig()).getPath("AutoWallPickup.path").relativeTo(drivetrain.getPose())),
          new ParallelCommandGroup(
            new AutoDrive(drivetrain, 108),
            new InstantCommand(()-> intake.setPercent(1), intake)
          ).andThen(new InstantCommand(()->intake.setPercent(0), intake)),
          new RamseteAuto(drivetrain, new Trajectories(drivetrain.getConfig()).getPath("AutoWallReturn.path").relativeTo(drivetrain.getPose())),
          new PrepShoot(Utils.findAngle(lidar), 20000, shooter),
          new ShootBalls()
        );
        //Getting balls from the trench
        case "Trench":
        return new SequentialCommandGroup(
          new PrepShoot(Utils.findAngle(lidar), 20000, shooter),
          new ShootBalls(),
          new RamseteAuto(drivetrain, new Trajectories(drivetrain.getConfig()).getPath("AutoTrench.path").relativeTo(drivetrain.getPose())),
          new ParallelCommandGroup(
            new AutoDrive(drivetrain, 108),
            new InstantCommand(()-> intake.setPercent(1), intake)
          ).andThen(new InstantCommand(()->intake.setPercent(0), intake)),
          new RamseteAuto(drivetrain, new Trajectories(drivetrain.getConfig()).getPath("AutoTrenchReturn.path").relativeTo(drivetrain.getPose())),
          new PrepShoot(Utils.findAngle(lidar), 20000, shooter),
          new ShootBalls()
        );
        //Just shooting
        default:
          return new SequentialCommandGroup(
            new PrepShoot(Utils.findAngle(lidar), 20000, shooter),
            new ShootBalls()
          );
    }
  }
}
