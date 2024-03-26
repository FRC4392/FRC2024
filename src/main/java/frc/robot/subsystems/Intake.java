// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public enum IntakeSpeeds{

    kIntakeSpeed(1),
    kIntakeSlowSpeed(.75),
    kOuttakeSpeed(-0.5),
    kStopSpeed(0);

    public final double speed;

    private IntakeSpeeds(double speeds){
      this.speed = speeds;
    }

  }

  public enum FeederSpeeds{

    kStopSpeed(0),
    kFeedSpeed(1),
    kInfeedSpeed(.25),
    kOutfeedSpeed(-.5),
    kHumanFeedSpeed(-.25),
    kReverseFeedSpeed(-.2);

    public final double speed;

    private FeederSpeeds(double speeds){
      this.speed = speeds;
    }

  }

  public enum IntakeState{

    kIntaking,
    kSlowIntaking,
    kOuttaking,
    kStopped,
    kFeeding,
    kSpitting,
    kHumanTaking;

  }

  public enum OccupancyState{
    kEmpty,
    kPartialPickup,
    kFull;
  }

  private CANSparkMax intakeMotor1 = new CANSparkMax(21, MotorType.kBrushless);
  private CANSparkMax intakeMotor2 = new CANSparkMax(22, MotorType.kBrushless);
  private CANSparkMax feederMotor = new CANSparkMax(33, MotorType.kBrushless);
  private DigitalInput feederLimit = new DigitalInput(0);
  private DigitalInput gapSwitch = new DigitalInput(1);
  private DigitalInput entrySwitch = new DigitalInput(2);
  private IntakeState state = IntakeState.kStopped;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor1.restoreFactoryDefaults();
    intakeMotor2.restoreFactoryDefaults();

    intakeMotor1.setSmartCurrentLimit(80);
    intakeMotor1.setIdleMode(IdleMode.kBrake);
    intakeMotor1.setInverted(true);

    intakeMotor2.setSmartCurrentLimit(80);
    intakeMotor2.setIdleMode(IdleMode.kBrake);
    intakeMotor2.setInverted(false);

    intakeMotor2.follow(intakeMotor1, true);

    intakeMotor1.burnFlash();
    intakeMotor2.burnFlash();

    feederMotor.restoreFactoryDefaults();

    feederMotor.setSmartCurrentLimit(60);
    feederMotor.setIdleMode(IdleMode.kBrake);
    feederMotor.setInverted(true);

    feederMotor.burnFlash();
  }

  public void setFeederMotorSpeed(double speed) {
    feederMotor.set(speed);
  }

  public void setIntakeMotorSpeed(double speed) {
    intakeMotor1.set(speed);
    //intakeMotor2.follow(intakeMotor1, true);
  }

  public void setIntakeSpeed(IntakeSpeeds speed){
    setIntakeMotorSpeed(speed.speed);
  }

  public void setFeederSpeed(FeederSpeeds speed){
    setFeederMotorSpeed(speed.speed);
  }

  public void stopFeed() {
    feederMotor.set(FeederSpeeds.kStopSpeed.speed);
  }

  public void stopIntake() {
    intakeMotor1.set(IntakeSpeeds.kStopSpeed.speed);
    intakeMotor2.set(IntakeSpeeds.kStopSpeed.speed);
  }

  public void setIntakeSpeed(IntakeSpeeds speed, IntakeSpeeds speed2) {
    if (getShooterSensor() && !getGap()) {
      intakeMotor1.set(speed.speed);
      intakeMotor2.set(speed.speed);
      feederMotor.set(speed2.speed);
    } else if (getShooterSensor() && getGap()) {
      intakeMotor1.set(speed.speed/2);
      intakeMotor2.set(speed.speed/2);
      feederMotor.set(speed2.speed/3);
    }else {
      stop();
    }
  }

  public void stop() {
    stopFeed();
    stopIntake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Shooter Limit", getShooterSensor());
    SmartDashboard.putBoolean("Gap Switch", getGap());
  }

  public IntakeState getState(){
    return this.state;
  }

  public OccupancyState getOccupancy(){
    if (!getShooterSensor()) {
      return OccupancyState.kFull;
    } else if (getGap() || getEntrySwitch()){
      return OccupancyState.kPartialPickup;
    } else {
      return OccupancyState.kEmpty;
    }
  }

  public Command intakeCommand(){
    return this.runEnd(()->{
      if (getShooterSensor() && !getGap()) {
        setIntakeSpeed(IntakeSpeeds.kIntakeSpeed);
        setFeederSpeed(FeederSpeeds.kFeedSpeed);
        state = IntakeState.kIntaking;
      } else if (getShooterSensor() && getGap()){
        setIntakeSpeed(IntakeSpeeds.kIntakeSlowSpeed);
        setFeederSpeed(FeederSpeeds.kInfeedSpeed);
        state = IntakeState.kSlowIntaking;
      } else {
        state = IntakeState.kStopped;
        stop();
      }
    }, ()->{{
      state = IntakeState.kStopped;
      stop();
    }});
  }

  public Command outtakeCommand(){
    return this.runEnd(()->{
      setIntakeSpeed(IntakeSpeeds.kOuttakeSpeed);
      setFeederSpeed(FeederSpeeds.kOutfeedSpeed);
      state = IntakeState.kOuttaking;
    }, ()->{
      state = IntakeState.kStopped;
      stop();
    });
  }

  public Command feedCommand() {
    return this.runEnd(() -> {
      setFeederSpeed(FeederSpeeds.kInfeedSpeed);
      state = IntakeState.kFeeding;
    }, () -> {
      state = IntakeState.kStopped;
      stopFeed();
    });
  }

  public Command reverseFeedCommand() {
    return this.runEnd(() -> {
      setFeederSpeed(FeederSpeeds.kReverseFeedSpeed);
      state = IntakeState.kFeeding;
    }, () -> {
      state = IntakeState.kStopped;
      stopFeed();
    });
  }

  public boolean getShooterSensor() {
    return feederLimit.get();
  }

  public boolean getGap(){
    return !gapSwitch.get();
  }

  public boolean getEntrySwitch(){
    return !entrySwitch.get();
  }

  public Command sptiCommand(){
    return this.runEnd(() -> {
      setFeederSpeed(FeederSpeeds.kFeedSpeed);
      setIntakeMotorSpeed(1);
      state = IntakeState.kSpitting;
    }, () -> {
      stop();
      state = IntakeState.kStopped;
    });
  }

  public Command humantakeCommand(){
    return this.runEnd(()->{
      if (getShooterSensor()) {
        setFeederSpeed(FeederSpeeds.kHumanFeedSpeed);
        state = IntakeState.kHumanTaking;
      } else {
        state = IntakeState.kStopped;
        stop();
      }
    }, ()->{{
      state = IntakeState.kStopped;
      stop();
    }});
  }
  
 }
