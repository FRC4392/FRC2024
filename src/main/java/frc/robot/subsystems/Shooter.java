// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax shooterMotor = new CANSparkMax(33, MotorType.kBrushless);
  private TalonFX shooter1Motor = new TalonFX(5);
  private TalonFX shooter2Motor = new TalonFX(2);
  public Shooter() {
shooterMotor.restoreFactoryDefaults();

shooterMotor.setSmartCurrentLimit(40);
shooterMotor.setIdleMode(IdleMode.kBrake);
shooterMotor.setInverted(true);

shooterMotor.burnFlash();

  }

  public void setShooterSpeed(double speed){
    shooterMotor.set(speed/2);
    shooter1Motor.set(speed);
    shooter2Motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
