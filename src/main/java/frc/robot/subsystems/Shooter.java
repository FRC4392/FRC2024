// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import java.util.function.DoubleSupplier;

import org.deceivers.drivers.LimelightHelpers;
import org.deceivers.util.interpolable.InterpolatingDouble;
import org.deceivers.util.interpolable.InterpolatingTreeMap;
import org.deceivers.util.interpolable.InverseInterpolable;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  public enum shooterSpeeds {
    kIntakeSpeed(.2),
    kOutfeedSpeed(-0.3),
    kBackfeedSpeed(-.15),
    kPivotSpeed(.1),
    kPivotBackSpeed(-.1),
    kSpitSpeed(.1),
    kStopSpeed(0);

    public final double speed;

    private shooterSpeeds(double speeds) {
      this.speed = speeds;
    }

  }

  
  private TalonFX shooter1Motor = new TalonFX(31);
  private TalonFX shooter2Motor = new TalonFX(32);
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
  private MotionMagicVoltage m_MotionMagicVoltage = new MotionMagicVoltage(0);

  public Shooter() {
    CurrentLimitsConfigs shooterCurrentLimits = new CurrentLimitsConfigs();

    shooterCurrentLimits.SupplyCurrentLimit = 40;
    shooterCurrentLimits.SupplyCurrentThreshold = 100;
    shooterCurrentLimits.SupplyTimeThreshold = 1;

    shooterCurrentLimits.StatorCurrentLimit = 80;

    shooterCurrentLimits.StatorCurrentLimitEnable = true;
    shooterCurrentLimits.SupplyCurrentLimitEnable = true;

    TalonFXConfiguration FlyWheelConfigs = new TalonFXConfiguration();

    FlyWheelConfigs.Feedback.SensorToMechanismRatio = 1.0/1.5;

    FlyWheelConfigs.Slot0.kV = 1/12.5; // velocity  speed from smart dashboard/11
    FlyWheelConfigs.Slot0.kP = .2; // proportional
    FlyWheelConfigs.Slot0.kI = 0; // integral
    FlyWheelConfigs.Slot0.kD = 0; // derivative
    
    FlyWheelConfigs.CurrentLimits = shooterCurrentLimits;

    shooter1Motor.getConfigurator().apply(FlyWheelConfigs);
    shooter2Motor.getConfigurator().apply(FlyWheelConfigs);
  }

  public void stop() {
    shooter1Motor.set(shooterSpeeds.kStopSpeed.speed);
    shooter2Motor.set(shooterSpeeds.kStopSpeed.speed);
  }

  public void setShooterSpeed(double velo) {
    shooter1Motor.setControl(m_voltageVelocity.withVelocity(velo));
    shooter2Motor.setControl(m_voltageVelocity.withVelocity(velo));
  }

  public void setHumanTake() {
    shooter1Motor.set(-.1);
    shooter2Motor.set(-.1);
  }

  public void setSpitSpeed(shooterSpeeds speed) {
    shooter1Motor.set(speed.speed);
    shooter2Motor.set(speed.speed);
  }

  

  //  public void setFeedwithPos(shooterSpeeds speed, double pos) {
  //   shooterMotor.set(speed.speed);
  //   shooterPivot.setControl(m_MotionMagicVoltage.withPosition(pos).withSlot(0));
  //   elevatorMotor.setControl(m_MotionMagicVoltage.withPosition(0).withSlot(0));
  // }


  //  public void setFeedandShooterSpeed(double speed, double velo) {
  //   shooterMotor.set(speed);
  //   shooter1Motor.setControl(m_voltageVelocity.withVelocity(velo));
  //   shooter2Motor.setControl(m_voltageVelocity.withVelocity(velo));
  // }

  // public void RunShooterAndFeeder(double speed){
  //   setFeedSpeed(shooterSpeeds.kFeedSpeed);
  //   setShooterSpeed(speed);
  // }

  

  

  // public Command feedWithPosCommand() {
  //   return this.runEnd(() -> setFeedwithPos(shooterSpeeds.kIntakeSpeed, 0), () -> stop());
  // }

  

  public Command backfeedCommand() {
    return this.runEnd(() -> setShooterSpeed(-10), () -> stop());
  }

  public Command runShooter(double velo) {
    return this.run(() -> setShooterSpeed(velo));
  }

  public Command spitCommand() {
    return this.runEnd(() -> setSpitSpeed(shooterSpeeds.kSpitSpeed), () -> stop());
  }

  public Command humanTakeCommand() {
    return this.runEnd(() -> setHumanTake(), () -> stop());
  }

  // public Command shootAndFeed(double speed){
  //   return this.runEnd(()->RunShooterAndFeeder(speed), () ->stop());
  // }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Shooter 1 Speed", shooter1Motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter 2 Speed", shooter2Motor.getVelocity().getValueAsDouble());
  }
}
