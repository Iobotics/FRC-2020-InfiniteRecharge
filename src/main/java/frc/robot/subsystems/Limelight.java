/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /**
   * Creates a new Limelight.
   */
  private  NetworkTable table; 
  private  NetworkTableInstance inst;
  private  NetworkTableEntry tv;
  private  NetworkTableEntry tx;
  private  NetworkTableEntry ty;
  private  NetworkTableEntry ta;

  private  double x = 0;
  private  double y = 0;
  private  double area = 0;

  public Limelight()  {
    inst = NetworkTableInstance.getDefault(); //setting up network tables for limelight
    table = inst.getTable("limelight");
    tx = table.getEntry("tx"); //getting information from the limelight
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    inst.startClientTeam(2438);
    inst.startDSClient();
    table.getEntry("ledMode").setNumber(1);
  }
  public boolean isTargetDetected() //returns true or false based of TV value from the limelight
  {
    if(this.getTV() == 0.0){
      return false;
    }else{
      return true;
    }
  }

  public double getTV(){ //gets the raw double TV value
    return tv.getDouble(0.0);
  }
  public double getTA() //returns area of the target on the limelight 
  {
    area = ta.getDouble(0.0); // mathwork needed base on height of bot and angle of camera
    return area;

  }
  public double getTX(){ //returns distance in degrees from the target
    x = tx.getDouble(0.0);
    return x;
  }

  public double getTY(){ //return the y displacement in degrees from the target
    y = ty.getDouble(0.0);
    return y;
  }

  public void printValues(){ //outprints all limelight values to the Smartdashboard 
    SmartDashboard.putNumber("LimelightX", this.getTX());
    SmartDashboard.putNumber("LimeilightY",this.getTY());
    SmartDashboard.putBoolean("Limelight TV", this.isTargetDetected());
    SmartDashboard.putNumber("LimeLightArea", this.getTA());
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
