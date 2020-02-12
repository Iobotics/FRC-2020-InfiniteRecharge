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

public class Machinelearning extends SubsystemBase {
  /**
   * Creates a new Machinelearning.
   */

   private NetworkTable table;
   private NetworkTableInstance inst;
   private NetworkTableEntry boxes;
   private NetworkTableEntry number;
   private NetworkTableEntry classes;

   double[] coordinates;
   double[] defaultValue = new double [0];

  public Machinelearning() {
    inst = NetworkTableInstance.getDefault(); //setting up network tables for limelight
    table = inst.getTable("ML");
    boxes = table.getEntry("boxes");
    number = table.getEntry("nb_objects");
    classes = table.getEntry("object_classes");
  
    inst.startClientTeam(2438);
    inst.startDSClient();

  }

 public boolean isPowerCellDetected()
 {
   if(this.getTargetNumber()> 0)
   {
     return true;
   }
   return false;
 }

 public String getClassName()
 {
   return classes.getString("None");
 }
 public double getTargetNumber()
 {
   return number.getDouble(0.0);
 }
 public void printValues()
 {
  SmartDashboard.putNumber("DetectedNumber", this.getTargetNumber());
  SmartDashboard.putString("ClassName", this.getClassName());


 }
  @Override
  public void periodic() {
    coordinates = boxes.getDoubleArray(defaultValue);
    SmartDashboard.putNumberArray("coordinates", coordinates);
    // This method will be called once per scheduler run
  }
}
