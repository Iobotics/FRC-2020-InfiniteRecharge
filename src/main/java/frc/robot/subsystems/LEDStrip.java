/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

public class LEDStrip extends SubsystemBase {
  /**
   * Creates a new LEDStrip.
   */

  private final I2C arduino;
  private final I2C.Port port = Port.kMXP;

  private boolean enable = true;

  public enum LEDColor {
    Red(255, 0, 0), Orange(255, 165, 0), Yellow(255, 255, 0), Green(0, 255, 0), Blue(0, 0, 255), Indigo(0, 128, 255), Violet(127, 0, 255), Off(0, 0, 0);

    private int[] values = {0,0,0};

    private LEDColor(int r, int g, int b) {
      values[0] = r;
      values[1] = g;
      values[2] = b;
    }
  }

  public LEDStrip() {
    arduino = new I2C(port, RobotMap.kArduino);
  }

  public void setEnable(final boolean enableVal) {
    enable = enableVal;
  }

  public void setColorAlliance (Alliance alliance) {
    if (alliance == Alliance.Blue) {
      setColor(LEDColor.Blue);
    } else {
      setColor(LEDColor.Red);
    }
  }

  public void setColor(final LEDColor color) {
   setCustomColor(color.values); 
  }

  /**
   * Write a custom RGB value to the LED Strip
   * 
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   * @return If the command was executed, true for execution
   */
  public boolean setCustomColor(final int r, final int g, final int b) {
    if (arduino.write(0, r) && arduino.write(0, g) && arduino.write(0, b) && enable) {
      return true;
    }
    return false;
  }

  /**
   * Internal Function to process enums into RGB values
   * @param values the array from the LEDColors enum (Should be 3 values)
   */
  private void setCustomColor(int[] values) {
    arduino.write(0, values[0]);
    arduino.write(0, values[1]);
    arduino.write(0, values[2]);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
