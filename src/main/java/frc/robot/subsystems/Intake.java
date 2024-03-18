// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public enum IntakeSpeeds{

    kIntakeSpeed(1),
    kOuttakeSpeed(-0.3),
    kStopSpeed(0),
    kFeedSpeed(.3),
    kInfeedSpeed(.15),
    kOutfeedSpeed(-.5);

    public final double speed;

    private IntakeSpeeds(double speeds){
      this.speed = speeds;
    }

  }

  private CANSparkMax intakeMotor = new CANSparkMax(21, MotorType.kBrushless);
  private CANSparkMax feederMotor = new CANSparkMax(33, MotorType.kBrushless);
  private SparkLimitSwitch feederLimit;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setSmartCurrentLimit(60);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    intakeMotor.burnFlash();

    feederMotor.restoreFactoryDefaults();

    feederMotor.setSmartCurrentLimit(40);
    feederMotor.setIdleMode(IdleMode.kBrake);
    feederMotor.setInverted(true);

    feederLimit = feederMotor.getForwardLimitSwitch(Type.kNormallyOpen);

    feederLimit.enableLimitSwitch(false);

    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 10);
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 10);
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 10);

    feederMotor.burnFlash();
  }

  public void setFeedSpeed() {
    feederMotor.set(IntakeSpeeds.kFeedSpeed.speed);
  }

  public void stopFeed() {
    feederMotor.set(IntakeSpeeds.kStopSpeed.speed);
  }

  public void setIntakeSpeed(IntakeSpeeds speed, IntakeSpeeds speed2) {
    if (getShooterSensor()) {
      intakeMotor.set(speed.speed);
      feederMotor.set(speed2.speed);
    } else {
      stop();
    }
    
  }

  public void stopIntake() {
    intakeMotor.set(IntakeSpeeds.kStopSpeed.speed);
  }

  public void stop() {
    intakeMotor.set(0);
    feederMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Shooter Limit", getShooterSensor());
  }

  public Command intakeCommand(){
    return this.runEnd(()->setIntakeSpeed(IntakeSpeeds.kIntakeSpeed, IntakeSpeeds.kInfeedSpeed), ()->stop());
  }

  public Command outtakeCommand(){
    return this.runEnd(()->setIntakeSpeed(IntakeSpeeds.kOuttakeSpeed, IntakeSpeeds.kOutfeedSpeed), ()->stop());
  }

  public void setFeedSpeed(IntakeSpeeds speed) {
    feederMotor.set(speed.speed);
  }

  public Command feedCommand() {
    return this.runEnd(() -> setFeedSpeed(IntakeSpeeds.kInfeedSpeed), () -> stopFeed());
  }

  public Command feedFastCommand(){
    return this.runEnd(() -> setFeedSpeed(IntakeSpeeds.kFeedSpeed), () ->stopFeed());
  }

  public Command outfeedCommand() {
    return this.runEnd(() -> setFeedSpeed(IntakeSpeeds.kOutfeedSpeed), () -> stopFeed());
  }

  public boolean getShooterSensor() {
    return !feederLimit.isPressed();
  }

//   public Command intakeandFeedCommand() {
//     return this.runEnd(() -> {
//       setFeedSpeed(IntakeSpeeds.kFeedSpeed);
//       setIntakeSpeed(1);
//     }, () -> {
//       stop();
//       stopFeed();
//     });
//   }
  
 }
