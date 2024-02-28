// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  public enum shooterSpeeds{

    kFeedSpeed(.3),
    kOutfeedSpeed(-0.3),
    kBackfeedSpeed(-.15),
    kPivotSpeed(.1),
    kPivotBackSpeed(-.1),
    kSpitSpeed(.1),
    kStopSpeed(0);

    public final double speed;

    private shooterSpeeds(double speeds){
      this.speed = speeds;
    }

  }

  private CANSparkMax shooterMotor = new CANSparkMax(33, MotorType.kBrushless);
  CANifier shooterCanifier = new CANifier(35);
  private TalonFX shooter1Motor = new TalonFX(31);
  private TalonFX shooter2Motor = new TalonFX(32);
  private TalonFX shooterPivot = new TalonFX(34);
  private TalonFX elevatorMotor = new TalonFX(41);
  
  public Shooter() {
  shooterMotor.restoreFactoryDefaults();

  shooterMotor.setSmartCurrentLimit(40);
  shooterMotor.setIdleMode(IdleMode.kBrake);
  shooterMotor.setInverted(true);

  shooterMotor.burnFlash();

  shooterPivot.setNeutralMode(NeutralModeValue.Brake);
  elevatorMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  // public void setLEDColor(double R, double G, double B) {
  //       shooterCanifier.setLEDOutput(G, LEDChannel.LEDChannelC);
  //       shooterCanifier.setLEDOutput(B, LEDChannel.LEDChannelA);
  //       shooterCanifier.setLEDOutput(R, LEDChannel.LEDChannelB);
  //      }

  public void stopFeed(){
    shooterMotor.set(shooterSpeeds.kStopSpeed.speed);
  }

  public void stopPivot(){
    shooterPivot.set(shooterSpeeds.kStopSpeed.speed);
  }

  public void stopShooter(){
    shooter1Motor.set(shooterSpeeds.kStopSpeed.speed);
    shooter2Motor.set(shooterSpeeds.kStopSpeed.speed);
  }

  public void stopElevator ()  {
    elevatorMotor.set(0);
  }

  public void stop(){
    shooter1Motor.set(shooterSpeeds.kStopSpeed.speed);
    shooter2Motor.set(shooterSpeeds.kStopSpeed.speed);
    shooterMotor.set(shooterSpeeds.kStopSpeed.speed);
  }

  public void setShooterSpeed(shooterSpeeds speed){
    shooter1Motor.set(speed.speed);
    shooter2Motor.set(speed.speed);
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

  
 public void setPivotSpeed(shooterSpeeds speed){
    shooterPivot.set(speed.speed);
  }
  
  public void setFeedSpeed(shooterSpeeds speed){
    shooterMotor.set(speed.speed);
  }

  public void setElevateSpeed (double speed) {
    elevatorMotor.set(speed);
  }

  public boolean getShooterSensor(){
    return shooterCanifier.getGeneralInput(GeneralPin.QUAD_A);
  }

  public Command feedCommand(){
    return this.runEnd(()->setFeedSpeed(shooterSpeeds.kFeedSpeed), ()->stopFeed());
  }

  public Command outfeedCommand(){
    return this.runEnd(()->setFeedSpeed(shooterSpeeds.kOutfeedSpeed), ()->stopFeed());
  }

  public Command backfeedCommand(){
    return this.runEnd(()->setShooterSpeed(shooterSpeeds.kBackfeedSpeed), ()->stopShooter());
  }

  public Command runShooter(DoubleSupplier speed){
    return this.runEnd(()->setShotSpeed(speed.getAsDouble()), ()->stopShooter());
  }

  public Command pivotCommand(){
    return this.runEnd(()->setPivotSpeed(shooterSpeeds.kPivotSpeed), ()->stopPivot());
  }

  public Command pivotBackCommand(){
    return this.runEnd(()->setPivotSpeed(shooterSpeeds.kPivotBackSpeed), ()->stopPivot());
  }

  public Command spitCommand(){
    return this.runEnd(()->setSpitSpeed(shooterSpeeds.kFeedSpeed,shooterSpeeds.kSpitSpeed), ()->stop());
  }

  public Command ElevateCommand(DoubleSupplier speed){
    return this.runEnd(()->setElevateSpeed(speed.getAsDouble()), ()->stopElevator());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
