// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  public enum shooterSpeeds {

    kFeedSpeed(.3),
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

  private CANSparkMax shooterMotor = new CANSparkMax(33, MotorType.kBrushless);
  CANifier shooterCanifier = new CANifier(35);
  private TalonFX shooter1Motor = new TalonFX(31);
  private TalonFX shooter2Motor = new TalonFX(32);
  private TalonFX shooterPivot = new TalonFX(34);
  private TalonFX elevatorMotor = new TalonFX(41);
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
  private MotionMagicVoltage m_MotionMagicVoltage = new MotionMagicVoltage(0);

  public Shooter() {
    shooterMotor.restoreFactoryDefaults();

    shooterMotor.setSmartCurrentLimit(40);
    shooterMotor.setIdleMode(IdleMode.kBrake);
    shooterMotor.setInverted(true);

    shooterMotor.burnFlash();

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
    FlyWheelConfigs.Slot0.kP = .4; // proportional
    FlyWheelConfigs.Slot0.kI = 0; // integral
    FlyWheelConfigs.Slot0.kD = 0; // derivative
    
    FlyWheelConfigs.CurrentLimits = shooterCurrentLimits;

    shooter1Motor.getConfigurator().apply(FlyWheelConfigs);
    shooter2Motor.getConfigurator().apply(FlyWheelConfigs);

    
    SoftwareLimitSwitchConfigs pivotSoftLimits = new SoftwareLimitSwitchConfigs();
    
    pivotSoftLimits.ForwardSoftLimitThreshold = .12;
    pivotSoftLimits.ReverseSoftLimitThreshold = .04;
    
    pivotSoftLimits.ForwardSoftLimitEnable = true;
    pivotSoftLimits.ReverseSoftLimitEnable = true;

    MotionMagicConfigs pivMotionMagicConfigs = new MotionMagicConfigs();

    pivMotionMagicConfigs.MotionMagicAcceleration = 1;
    pivMotionMagicConfigs.MotionMagicCruiseVelocity = .2;
    pivMotionMagicConfigs.MotionMagicJerk = 0;

    TalonFXConfiguration PivotConfigs = new TalonFXConfiguration();

    PivotConfigs.Feedback.SensorToMechanismRatio = 50/12 * 50/14 * 117/10;

    PivotConfigs.Slot0.kV = 1/.05; // velocity
    PivotConfigs.Slot0.kA = 0; // acceleration
    PivotConfigs.Slot0.kG = 0; // gravity

    PivotConfigs.Slot0.kP = 1000; // proportional
    PivotConfigs.Slot0.kI = 0; // integral
    PivotConfigs.Slot0.kD = 0; // derivative

    PivotConfigs.CurrentLimits = shooterCurrentLimits;
    PivotConfigs.SoftwareLimitSwitch = pivotSoftLimits;
    PivotConfigs.MotionMagic = pivMotionMagicConfigs;

    TalonFXConfiguration ElevatorConfigs = new TalonFXConfiguration();

     SoftwareLimitSwitchConfigs elevatorSoftLimits = new SoftwareLimitSwitchConfigs();
    
    elevatorSoftLimits.ForwardSoftLimitThreshold = 3;
    elevatorSoftLimits.ReverseSoftLimitThreshold = 0;
    
    elevatorSoftLimits.ForwardSoftLimitEnable = true;
    elevatorSoftLimits.ReverseSoftLimitEnable = true;

    MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();

    elevatorMotionMagicConfigs.MotionMagicAcceleration = 4;
    elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = 2;
    elevatorMotionMagicConfigs.MotionMagicJerk = 0;

     ElevatorConfigs.Feedback.SensorToMechanismRatio = 52/12 * 56/14;

    ElevatorConfigs.Slot0.kV = 2/1.1; // velocity
    ElevatorConfigs.Slot0.kA = 0; // acceleration
    ElevatorConfigs.Slot0.kG = 0; // gravity

    ElevatorConfigs.Slot0.kP = 40; // proportional
    ElevatorConfigs.Slot0.kI = 0; // integral
    ElevatorConfigs.Slot0.kD = 0; // derivative

    ElevatorConfigs.CurrentLimits = shooterCurrentLimits;
    ElevatorConfigs.MotionMagic = elevatorMotionMagicConfigs;
    ElevatorConfigs.SoftwareLimitSwitch = elevatorSoftLimits;

    shooterPivot.getConfigurator().apply(PivotConfigs);
    elevatorMotor.getConfigurator().apply(ElevatorConfigs);

    shooterPivot.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void stopFeed() {
    shooterMotor.set(shooterSpeeds.kStopSpeed.speed);
  }

  public void stopPivot() {
    shooterPivot.set(shooterSpeeds.kStopSpeed.speed);
  }

  public void setPivotPos(double pos) {
    shooterPivot.setControl(m_MotionMagicVoltage.withPosition(pos).withSlot(0));
  }

  public void setElevatorPos(double pos) {
    elevatorMotor.setControl(m_MotionMagicVoltage.withPosition(pos).withSlot(0));
  }

  public void stopShooter() {
    shooter1Motor.set(shooterSpeeds.kStopSpeed.speed);
    shooter2Motor.set(shooterSpeeds.kStopSpeed.speed);
  }

  public void stopElevator() {
    elevatorMotor.set(0);
  }

  public void stop() {
    shooter1Motor.set(shooterSpeeds.kStopSpeed.speed);
    shooter2Motor.set(shooterSpeeds.kStopSpeed.speed);
    shooterMotor.set(shooterSpeeds.kStopSpeed.speed);
  }

  public void setShooterVoltage(double volts){
    VoltageOut voltage = new VoltageOut(volts);
    shooterPivot.setControl(voltage);
  }

  public void setElevatorVoltage(double volts){
    VoltageOut voltage = new VoltageOut(volts);
    elevatorMotor.setControl(voltage);
  }

  public void setShooterSpeed(double velo) {
    shooter1Motor.setControl(m_voltageVelocity.withVelocity(velo));
    shooter2Motor.setControl(m_voltageVelocity.withVelocity(velo));
  }

  public void setShotSpeed(double speed) {
    shooter1Motor.set(speed);
    shooter2Motor.set(speed);
  }

  public void setSpitSpeed(shooterSpeeds speedF, shooterSpeeds speedS) {
    shooter1Motor.set(speedS.speed);
    shooter2Motor.set(speedS.speed);
    shooterMotor.set(speedF.speed);
  }

  public void setPivotSpeed(double velo) {
    shooterPivot.setControl(m_voltageVelocity.withVelocity(velo));
  }

  public void setFeedSpeed(shooterSpeeds speed) {
    shooterMotor.set(speed.speed);
  }

   public void setFeedandShooterSpeed(double speed, double velo) {
    shooterMotor.set(speed);
    shooter1Motor.setControl(m_voltageVelocity.withVelocity(velo));
    shooter2Motor.setControl(m_voltageVelocity.withVelocity(velo));
  }

  public void setElevateSpeed(double speed) {
    elevatorMotor.setControl(m_voltageVelocity.withVelocity(speed));
  }

  public boolean getShooterSensor() {
    return shooterCanifier.getGeneralInput(GeneralPin.QUAD_A);
  }

  public Command feedCommand() {
    return this.runEnd(() -> setFeedandShooterSpeed(.2,100), () -> stop());
  }

  public Command outfeedCommand() {
    return this.runEnd(() -> setFeedSpeed(shooterSpeeds.kOutfeedSpeed), () -> stopFeed());
  }

  public Command backfeedCommand() {
    return this.runEnd(() -> setShooterSpeed(-10), () -> stopShooter());
  }

  public Command runShooter() {
    return this.runEnd(() -> setShooterSpeed(100), () -> stopShooter());
  }

  public Command pivotCommand() {
    return this.runEnd(() -> setPivotSpeed(-.3), () -> stopPivot());
  }

  public Command pivotBackCommand() {
    return this.runEnd(() -> setPivotSpeed(.3), () -> stopPivot());
  }

  public Command spitCommand() {
    return this.runEnd(() -> setSpitSpeed(shooterSpeeds.kFeedSpeed, shooterSpeeds.kSpitSpeed), () -> stop());
  }

  public Command ElevateCommand() {
    return this.runEnd(() -> setElevateSpeed(1), () -> stopElevator());
  }

  public Command DeElevateCommand() {
    return this.runEnd(() -> setElevateSpeed(-1), () -> stopElevator());
  }

  public Command CalibrateKs(double volts){
    return this.runEnd(() -> setShooterVoltage(volts), () -> stopPivot());
  }

  public Command CalibrateKe(double volts){
    return this.runEnd(() -> setElevatorVoltage(volts), () -> stopElevator());
  }

  public Command pivotToPosCommand(double pos) {
    return this.run(() -> setPivotPos(pos));
  }

  public Command elevateToPosCommand(double pos) {
    return this.run(() -> setElevatorPos(pos));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Shooter 1 Speed", shooter1Motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter 2 Speed", shooter2Motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Speed", elevatorMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Speed", shooterPivot.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("PivotPos", shooterPivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("ElevatorPos", elevatorMotor.getPosition().getValueAsDouble()); //up 0.015 down .13
  }
}
