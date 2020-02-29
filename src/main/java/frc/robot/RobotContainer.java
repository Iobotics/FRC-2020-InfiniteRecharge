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
import frc.robot.commands.AutoHopper;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final Hopper hopper = new Hopper();
  private final Shooter shooter = new Shooter();
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
    
    

    //Default Limelight command 
    limelight.setDefaultCommand(new RunCommand(() -> limelight.printValues(), limelight));
    hopper.setDefaultCommand(new AutoHopper(hopper, 0.5));
    SmartDashboard.putNumber("Auto Number", 0);
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
    
    //Test Commands (delete later)
    new JoystickButton(joystick1, OIConstants.kTest).whenPressed(
      new InstantCommand(()-> shooter.setHoodAbsolute(0), shooter));

    new JoystickButton(joystick2, OIConstants.kTest).whileHeld(
      new RunCommand(() -> shooter.setVelocity(2000), shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    if(SmartDashboard.getNumber("Auto Number", 0) == 0){
      return new SequentialCommandGroup(
      new AutoDrive(drivetrain, 130.36), 
      new Auto(gyro, -135, gyro.getAngle(), drivetrain) //input the actual measured angle theta later as well as add shoot and intake commands
      );
    }else if(SmartDashboard.getNumber("Auto Number", 0) == 1){
      return new SequentialCommandGroup(
        new AutoDrive(drivetrain, 120),
        new Auto(gyro, -90, gyro.getAngle(), drivetrain),
        new AutoDrive(drivetrain, 120),
        new Auto(gyro, -90, gyro.getAngle(), drivetrain),
        new AutoDrive(drivetrain, 120),
        new Auto(gyro, -90, gyro.getAngle(), drivetrain),
        new AutoDrive(drivetrain, 120),
        new Auto(gyro, -90, gyro.getAngle(), drivetrain));
    }else if(SmartDashboard.getNumber("Auto Number", 0) == 2){
      return new SequentialCommandGroup(
        new AutoDrive(drivetrain, 10),
        new Auto(gyro, 180, gyro.getAngle(), drivetrain),
        //shoot ball here with shooter at angle theta
        new Auto(gyro, 180, gyro.getAngle(), drivetrain));
    }else{
      return new SequentialCommandGroup(
        //shoot balls
        new Auto(gyro, 180, gyro.getAngle(), drivetrain),
        new AutoDrive(drivetrain, 20));
    }
    
    //new AutoAlign(limelight, drivetrain)
    //AutoAlign(limelight, drivetrain);
    //return new AutoDrive(drivetrain, 120);
  }
}
