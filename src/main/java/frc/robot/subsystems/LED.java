// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

public class LED extends SubsystemBase {
  private CANifier remoteIO = new CANifier(35);
  private AddressableLED leds = new AddressableLED(0);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(33);
  int m_rainbowFirstPixelHue = 1;
  
  public void setLEDColor(double R, double G, double B) {
    remoteIO.setLEDOutput(G, LEDChannel.LEDChannelC);
    remoteIO.setLEDOutput(R, LEDChannel.LEDChannelA);
    remoteIO.setLEDOutput(B, LEDChannel.LEDChannelB);
  }

  public LED() {
    leds.setLength(buffer.getLength());
    setLedCube();
    leds.start();
  }

  @Override
  public void periodic() {
    rainbow();
  }

  public Command setLedOccupied(){
    return this.runEnd(()->setLEDColor(0,0,1), ()->setLEDColor(1,.2,0));
  }

  public Command setLedRed() {
    return this.runEnd(()->setLEDColor(100,0,0), ()->setLEDColor(100,0,0));
  }

  public void setLedCube() {
    for (var i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      buffer.setRGB(i, 0, 0, 255);
    }
    leds.setData(buffer);
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < buffer.getLength(); i++) {

      if (i % 2 == 1) {
        if (m_rainbowFirstPixelHue < 5){
          buffer.setRGB(i, 255, 64, 0);
        } else {
          buffer.setRGB(i, 0, 0, 255);
        }
      } else {
        if (m_rainbowFirstPixelHue < 5){
          buffer.setRGB(i, 0, 0, 255);
        } else {
          buffer.setRGB(i, 255, 64, 0);
        }
      }
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue +=1;
    // Check bounds
    if (m_rainbowFirstPixelHue > 10){
      m_rainbowFirstPixelHue = 1;
    }

    leds.setData(buffer);
  }

}
