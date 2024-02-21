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

  public enum IntakeSpeeds{

    kIntakeSpeed(1),
    kOuttakeSpeed(-0.5),
    kStopSpeed(0);

    public final double speed;

    private IntakeSpeeds(double speeds){
      this.speed = speeds;
    }

  }

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

  public void setIntakeSpeed(IntakeSpeeds speed){
    intakeMotor.set(speed.speed);
  }

  public void stop(){
    intakeMotor.set(IntakeSpeeds.kStopSpeed.speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command intakeCommand(){
    return this.runEnd(()->setIntakeSpeed(IntakeSpeeds.kIntakeSpeed), ()->stop());
  }

  public Command outtakeCommand(){
    return this.runEnd(()->setIntakeSpeed(IntakeSpeeds.kOuttakeSpeed), ()->stop());
  }
}
