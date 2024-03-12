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
  
  public LED() {
    leds.setLength(buffer.getLength());
    leds.start();
  }
  
  public void setShooterLED(double R, double G, double B) {
    remoteIO.setLEDOutput(G, LEDChannel.LEDChannelC);
    remoteIO.setLEDOutput(R, LEDChannel.LEDChannelA);
    remoteIO.setLEDOutput(B, LEDChannel.LEDChannelB);
  }

  public void setElevatorLED(int R, int G, int B) {
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, R, G, B);
    }
    leds.setData(buffer);
  }

  public void setAllLED(int R, int G, int B) {
    remoteIO.setLEDOutput(G/255, LEDChannel.LEDChannelC);
    remoteIO.setLEDOutput(R/255, LEDChannel.LEDChannelA);
    remoteIO.setLEDOutput(B/255, LEDChannel.LEDChannelB);

    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, R, G, B);
    }
    leds.setData(buffer);
  }

  @Override
  public void periodic() {
   // rainbow();
  }

  public Command setShooterLedOccupied(){
    return this.runEnd(()->setShooterLED(0,0,1), ()->setShooterLED(1,.2,0));
  }

  public Command setShooterLedRed() {
    return this.runEnd(()->setShooterLED(1,0,0), ()->setShooterLED(1,0,0));
  }

  public Command setShooterLedTracking() {
    return this.runEnd(()->setShooterLED(1,0,0), ()->setShooterLED(0,1,0));
  }

  public Command setLedsOccupied() {
    return this.runEnd(()->setAllLED(255,50,0), ()->setAllLED(0,0,255));
  }

  public Command setLedsPurple() {
    return this.runEnd(()->setAllLED(255,0,255), ()->setAllLED(0,0,255));
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
