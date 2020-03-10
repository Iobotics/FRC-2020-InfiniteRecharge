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
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.RobotMap;

public class LEDStrip extends SubsystemBase {
  /**
   * Creates a new LEDStrip.
   */

  private final I2C arduino;
  private final Port port = Port.kMXP;

  private boolean enable = true;

  public enum LEDColor {
    RED(255, 0, 0), ORANGE(255, 165, 0), YELLOW(255, 255, 0), GREEN(0, 255, 0), BLUE(0, 0, 255), INDIGO(0, 128, 255), VIOLET(127, 0, 255), OFF(0, 0, 0);

    private int[] values = {0,0,0};

    private LEDColor(int r, int g, int b) {
      values[0] = r;
      values[1] = g;
      values[2] = b;
    }
  }

  public enum LEDSector {
    HOPPER_SIDES(0, 121), SHOOTER(122, 178);

    private int[] values = {0,0};

    private LEDSector(int startLED, int endLED) {
      values[0] = startLED;
      values[1] = endLED;
    }
  }

  public LEDStrip() {
    arduino = new I2C(port, RobotMap.kArduino);
  }

  public void setEnable(final boolean enableVal) {
    enable = enableVal;
    if (!enable) {
      setColor(LEDColor.OFF);
    }
  }

  public void setColorAlliance (Alliance alliance) {
    if (alliance == Alliance.Blue) {
      setColor(LEDColor.BLUE);
    } else {
      setColor(LEDColor.RED);
    }
  }

  public void setColor(final LEDColor color) {
    setCustomColor(color.values, LEDConstants.kFirst, LEDConstants.kLast); 
  }

  public void setColor(LEDColor color, LEDSector sector) {
    setCustomColor(color.values, sector.values[0], sector.values[1]);
  }

  public void setColor(LEDColor color, int startLED, int endLED) {
    setCustomColor(color.values, startLED, endLED);
  }

  /**
   * Write a custom RGB value to the LED Strip
   * 
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   * @return If the command was executed, true for execution
   */
  public boolean setCustomColor(final int r, final int g, final int b, int startLED, int endLED) {
    if (enable) {
      if (arduino.write(0x00, r) && arduino.write(0x00, g) && arduino.write(0x00, b) && arduino.write(0x00, startLED) && arduino.write(0x00, endLED)) {
        return true;
      } else {
        return false;
      }
    }
    return false;
  }

  /**
   * Internal Function to process enums into RGB values
   * @param values the array from the LEDColors enum (Should be 3 values)
   */
  private void setCustomColor(int[] values, int startLED, int endLED) {
    if (enable) {
      arduino.write(0x00, values[0]);
      arduino.write(0x00, values[1]);
      arduino.write(0x00, values[2]);
      arduino.write(0x00, startLED);
      arduino.write(0x00, endLED);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
