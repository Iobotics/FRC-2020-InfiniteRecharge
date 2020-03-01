/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class Lidar extends SubsystemBase {
  /**
   * Creates a new Lidar.
   */
  private I2C lidar;

  private byte[] buffer;
  private byte[] delay;
  private byte[] test ={0,0,0};

  private long value;

  public Lidar() {
    buffer = new byte[2];
    delay = new byte[6];
    lidar = new I2C(Port.kMXP, 98);
  }

  public long getValue() {
    return value;
  }

  @Override
  public void periodic() {
    
    lidar.write(0x00, 0x04);
    lidar.read(0x01, 6, delay); 
    if(delay[0] == 1){
    } else {
      lidar.read(0x8f, 2, buffer);
      value = Integer.toUnsignedLong(buffer[0] << 8) + Byte.toUnsignedInt(buffer[1]);
      SmartDashboard.putNumber("Lidar", value);
    }
  }
}