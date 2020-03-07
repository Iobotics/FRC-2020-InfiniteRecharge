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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import java.lang.Math;

public class MachineLearning extends SubsystemBase {
  /**
   * Creates a new Machinelearning.
   */

   private NetworkTable table;
   private NetworkTableInstance inst;
   private NetworkTableEntry boxes;
   private NetworkTableEntry number;
   private double[] defaultValue = new double[] {0,0,0,0,0,0,0,0,0,0,0,0,0};
   private double[] coornidates = new double[]{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    //initialize variables
   public  double PWerror = 0; //angle of error in degrees
   private double y = 0;  //y coordinate
   private double Sx = 0; //scaled x coordinate relative to bot


  public MachineLearning() {
    inst = NetworkTableInstance.getDefault(); //setting up network tables for Machinelearning
    table = inst.getTable("ML");
    boxes = table.getEntry("boxes");
    number = table.getEntry("nb_objects");
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


 public double getTargetNumber()
 {
   return number.getDouble(0.0);
 }

 public double[] getCoordinate()
 {
    return boxes.getDoubleArray(defaultValue);
}

  


  public void printValues()
 {
   if(this.getCoordinate().length!=0) // Use length of coordinate array instead of getTargetNumber to avoid index exception 
   {
  coornidates=this.getCoordinate();
    }
    int n = this.findNearest(); // get id*4 of the nearest power cell
    double x1 = coornidates[n]-180;
    double y1 = 280-coornidates[n+1];
    double x2 = coornidates[n+2]-180;
    double y2 = 280-coornidates[n+3];
    double x = 0.5*(x1+x2);
  
    y = 0.5*(y1+y2);
    Sx = (1.248*(y2/260)+1)*x;  // Scaling X value relative to the bot (image rectification)
   
  SmartDashboard.putNumber("DetectedNumber", this.getTargetNumber());
  SmartDashboard.putNumber("PW:x1",x1);
  SmartDashboard.putNumber("PW:y1",y1);
  SmartDashboard.putNumber("PW:x2",x2);
  SmartDashboard.putNumber("PW:y2",y2);
  SmartDashboard.putNumber("PW:x",Sx); 
  SmartDashboard.putNumber("PW:y",y);
  SmartDashboard.putNumber("PW:AOE",this.giveError()); //angle of error
   }


  public int findNearest()
  {
    if(this.getTargetNumber()==1){
      return 0;
    }
    if(this.getCoordinate().length!=0)
   {
  coornidates=this.getCoordinate();
    }
    int nearestPC=0;
    for(int i=0; i<this.getCoordinate().length/4; i++) // looping and looking for the smallest y coordinate value
    {
      if(coornidates[i*4+1]+coornidates[i*4+3]>coornidates[nearestPC*4+1]+coornidates[nearestPC*4+3]) 
      nearestPC = i;
    }
    return nearestPC*4; 
  }


  public double giveError()
  {
    

    PWerror = Math.atan(Sx/(y+57))/2/3.14*180-0.5; //calculate angle of error
    return PWerror;
  }

  @Override
  public void periodic() {
    
    
    // This method will be called once per scheduler run
  }
}
