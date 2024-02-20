// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax shooterMotor = new CANSparkMax(33, MotorType.kBrushless);
  private TalonFX shooter1Motor = new TalonFX(31);
  private TalonFX shooter2Motor = new TalonFX(32);
  private TalonFX shooterPivot = new TalonFX(34);
  public Shooter() {
  shooterMotor.restoreFactoryDefaults();

  shooterMotor.setSmartCurrentLimit(40);
  shooterMotor.setIdleMode(IdleMode.kBrake);
  shooterMotor.setInverted(true);

  shooterMotor.burnFlash();

  shooterPivot.setNeutralMode(NeutralModeValue.Brake);

  }

  public void setShooterSpeed(double speed){
    shooter1Motor.set(speed);
    shooter2Motor.set(speed);
  }
 public void setPivotSpeed(double speed){
    shooterPivot.set(speed);
  }
  
  public void setFeedSpeed(double speed){
    shooterMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
