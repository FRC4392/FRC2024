// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.deceivers.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class SwerveState {
    private Rotation2d rotation2d;
    private double velocity;

    public SwerveState(Rotation2d rot, double vel){
        rotation2d = rot;
        velocity = vel;
    }

    public static SwerveState fromDegrees(double degrees, double velocity){
        return new SwerveState(Rotation2d.fromDegrees(degrees), velocity);
    }

    public static SwerveState fromRadians(double radians, double velocity){
        return new SwerveState(new Rotation2d(radians), velocity);
    }

    public double getDegrees(){
        return rotation2d.getDegrees();
    }

    public double getRadians(){
        return rotation2d.getRadians();
    }

    public double getVelocity(){
        return velocity;
    }
}
