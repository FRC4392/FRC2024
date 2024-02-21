// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor = new CANSparkMax(21, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.burnFlash();
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }

  public void stop(){
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command intakeCommand(){
    return this.runEnd(()->setIntakeSpeed(1), ()->stop());
  }
}
