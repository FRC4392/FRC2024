// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.deceivers.drivers.LimelightHelpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagLimelight extends SubsystemBase {
  /** Creates a new AprilTagLimelight. */
  public AprilTagLimelight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("limelight tx", LimelightHelpers.getTX("limelight-april"));
  }
}
