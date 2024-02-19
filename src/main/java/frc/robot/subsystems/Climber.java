// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private CANSparkMax climberMotor = new CANSparkMax(42, MotorType.kBrushless);
  public Climber() {
    climberMotor.restoreFactoryDefaults();

    climberMotor.setSmartCurrentLimit(40);
    climberMotor.setIdleMode(IdleMode.kBrake);
    
    climberMotor.burnFlash();
  }

  public void setClimberSpeed(double speed){
    climberMotor.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
