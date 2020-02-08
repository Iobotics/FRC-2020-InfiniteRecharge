/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OIConstants;
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

  private final AHRS gyro = new AHRS();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight();

  private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
  private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);

  private final XboxController xboxController = new XboxController(OIConstants.kXboxController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(new RunCommand(
        () -> drivetrain.setTank(Math.pow(-joystick1.getY(), 3), Math.pow(-joystick2.getY(), 3)), drivetrain));
    limelight.setDefaultCommand(new RunCommand(() -> limelight.printValues(), limelight));

  }

  public double getGyro() {
    return gyro.getAngle();
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick1, 2).whenPressed(new AutoAlign(limelight, drivetrain));
    new JoystickButton(joystick1, 1).whenPressed(new StartEndCommand(
        () -> drivetrain.setTank(SmartDashboard.getNumber("power", 1), SmartDashboard.getNumber("power", 1)),
        () -> drivetrain.setTank(0, 0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    /*return new SequentialCommandGroup(
      new AutoDrive(drivetrain, 130.36), 
      new Auto(gyro, -135, gyro.getAngle(), drivetrain) //input the actual measured angle theta later as well as add shoot and intake commands
      );*/
      return new SequentialCommandGroup(
        new AutoDrive(drivetrain, 120),
        new Auto(gyro, -90, gyro.getAngle(), drivetrain),
        new AutoDrive(drivetrain, 120),
        new Auto(gyro, -90, gyro.getAngle(), drivetrain),
        new AutoDrive(drivetrain, 120),
        new Auto(gyro, -90, gyro.getAngle(), drivetrain),
        new AutoDrive(drivetrain, 120),
        new Auto(gyro, -90, gyro.getAngle(), drivetrain));
      )
    //new AutoAlign(limelight, drivetrain)
    //AutoAlign(limelight, drivetrain);
    //return new AutoDrive(drivetrain, 120);
  }
}
