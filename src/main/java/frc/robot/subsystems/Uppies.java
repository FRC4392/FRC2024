// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Uppies extends SubsystemBase {
  /** Creates a new Climber. */

  private CANSparkMax wallMotor = new CANSparkMax(42, MotorType.kBrushless);
  private TalonFX climberMotor = new TalonFX(44);
  

  public Uppies() {
    wallMotor.restoreFactoryDefaults();

    wallMotor.setSmartCurrentLimit(40);
    wallMotor.setIdleMode(IdleMode.kBrake);

    wallMotor.burnFlash();

    climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setWallDriveSpeed(double speed){
    wallMotor.set(speed);
  }

  public void setClimbSpeed (double speed) {
    climberMotor.set(speed);
  }

  public void stopClimb () {
    climberMotor.set(0);
  }

  public void stopWall ()  {
    wallMotor.set(0);
  }

  public Command ClimbUpCommand(){
    return this.runEnd(()->setClimbSpeed(.5), ()->stopClimb());
  }

  public Command ClimbDownCommand(){
    return this.runEnd(()->setClimbSpeed(-.1), ()->stopClimb());
  }


  public Command WallDriveCommand(DoubleSupplier speed){
    return this.runEnd(()->setWallDriveSpeed(speed.getAsDouble()), ()->stopWall());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
