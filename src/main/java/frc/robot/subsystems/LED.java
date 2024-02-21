// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.CANifier.LEDChannel;

public class LED extends SubsystemBase {
  private CANifier remoteIO = new CANifier(35);
  
  public void setLEDColor(double R, double G, double B) {
    remoteIO.setLEDOutput(G, LEDChannel.LEDChannelC);
    remoteIO.setLEDOutput(B, LEDChannel.LEDChannelA);
    remoteIO.setLEDOutput(R, LEDChannel.LEDChannelB);
  }

  public LED() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
