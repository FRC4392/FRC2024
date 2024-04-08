// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.deceivers.drivers.LimelightHelpers;
import org.deceivers.util.interpolable.InterpolatingDouble;
import org.deceivers.util.interpolable.InterpolatingTreeMap;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

  private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> map = new InterpolatingTreeMap<>();
  static {
    map.put(new InterpolatingDouble(22.5), new InterpolatingDouble(.12));//40in
    //map.put(new InterpolatingDouble(12.25), new InterpolatingDouble(.1));
    map.put(new InterpolatingDouble(9.36), new InterpolatingDouble(.09));
    map.put(new InterpolatingDouble(.935), new InterpolatingDouble(.066));
    map.put(new InterpolatingDouble(-4.18), new InterpolatingDouble(.052));
    map.put(new InterpolatingDouble(-7.5), new InterpolatingDouble(.045));//135.5
    map.put(new InterpolatingDouble(-10.42), new InterpolatingDouble(.037));
    map.put(new InterpolatingDouble(-12.32), new InterpolatingDouble(.0325));
    map.put(new InterpolatingDouble(-13.84), new InterpolatingDouble(.0265));
    map.put(new InterpolatingDouble(-15.12), new InterpolatingDouble(.0225));
    map.put(new InterpolatingDouble(-16.0), new InterpolatingDouble(.025));
    map.put(new InterpolatingDouble(-18.31), new InterpolatingDouble(.018));
    //map.put(new InterpolatingDouble(7.0), new InterpolatingDouble(.082));
    //map.put(new InterpolatingDouble(0.0), new InterpolatingDouble(.063));
    //map.put(new InterpolatingDouble(-.38), new InterpolatingDouble(.059));
    //map.put(new InterpolatingDouble(-6.37), new InterpolatingDouble(.048));
    // map.put(new InterpolatingDouble(-8.185), new InterpolatingDouble(.045));
    // map.put(new InterpolatingDouble(-10.15), new InterpolatingDouble(.038));
    // map.put(new InterpolatingDouble(-12.39), new InterpolatingDouble(.0322));
    // map.put(new InterpolatingDouble(-13.47), new InterpolatingDouble(.0315));
    // map.put(new InterpolatingDouble(-15.67), new InterpolatingDouble(.02));
    // map.put(new InterpolatingDouble(-18.78), new InterpolatingDouble(.018));
  }

  public enum AimingState {
    kNotAiming,
    kAiming,
    kAimed,
    kTooFar;
  }

  /** Creates a new Superstructure. */

  private TalonFX shooterPivot = new TalonFX(34);
  private TalonFX elevatorMotor = new TalonFX(41);

  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
  private MotionMagicVoltage m_MotionMagicVoltage = new MotionMagicVoltage(0);

  private AimingState state = AimingState.kNotAiming;


  public Superstructure() {
    CurrentLimitsConfigs shooterCurrentLimits = new CurrentLimitsConfigs();

    shooterCurrentLimits.SupplyCurrentLimit = 40;
    shooterCurrentLimits.SupplyCurrentThreshold = 100;
    shooterCurrentLimits.SupplyTimeThreshold = 1;

    shooterCurrentLimits.StatorCurrentLimit = 80;

    shooterCurrentLimits.StatorCurrentLimitEnable = true;
    shooterCurrentLimits.SupplyCurrentLimitEnable = true;

    SoftwareLimitSwitchConfigs pivotSoftLimits = new SoftwareLimitSwitchConfigs();
    
    pivotSoftLimits.ForwardSoftLimitThreshold = .12;
    pivotSoftLimits.ReverseSoftLimitThreshold = .0;
    
    pivotSoftLimits.ForwardSoftLimitEnable = true;
    pivotSoftLimits.ReverseSoftLimitEnable = true;

    MotionMagicConfigs pivMotionMagicConfigs = new MotionMagicConfigs();

    pivMotionMagicConfigs.MotionMagicAcceleration = 5;
    pivMotionMagicConfigs.MotionMagicCruiseVelocity = .6;
    pivMotionMagicConfigs.MotionMagicJerk = 0;

    MotorOutputConfigs pivOutputConfigs = new MotorOutputConfigs();

    pivOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);

    TalonFXConfiguration PivotConfigs = new TalonFXConfiguration();

    PivotConfigs.Feedback.SensorToMechanismRatio = 50/12 * 50/14 * 117/10;

    PivotConfigs.Slot0.kV = 1/.05; // velocity
    PivotConfigs.Slot0.kA = 0; // acceleration
    PivotConfigs.Slot0.kG = 0; // gravity

    PivotConfigs.Slot0.kP = 1200; // proportional
    PivotConfigs.Slot0.kI = 0; // integral
    PivotConfigs.Slot0.kD = 0; // derivative

    PivotConfigs.CurrentLimits = shooterCurrentLimits;
    PivotConfigs.SoftwareLimitSwitch = pivotSoftLimits;
    PivotConfigs.MotionMagic = pivMotionMagicConfigs;
    PivotConfigs.MotorOutput = pivOutputConfigs;

    TalonFXConfiguration ElevatorConfigs = new TalonFXConfiguration();

     SoftwareLimitSwitchConfigs elevatorSoftLimits = new SoftwareLimitSwitchConfigs();
    
    elevatorSoftLimits.ForwardSoftLimitThreshold = 3;
    elevatorSoftLimits.ReverseSoftLimitThreshold = 0;
    
    elevatorSoftLimits.ForwardSoftLimitEnable = true;
    elevatorSoftLimits.ReverseSoftLimitEnable = true;

        MotionMagicConfigs elevatorMotionMagicConfigs = new MotionMagicConfigs();

    elevatorMotionMagicConfigs.MotionMagicAcceleration = 6;
    elevatorMotionMagicConfigs.MotionMagicCruiseVelocity = 3;
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

    // shooterPivot.optimizeBusUtilization();
    elevatorMotor.getConfigurator().apply(ElevatorConfigs);
    // elevatorMotor.optimizeBusUtilization();

    shooterPivot.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter is Aiming", state == AimingState.kAimed || state == AimingState.kAiming);
    SmartDashboard.putNumber("Elevator Speed", elevatorMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Speed", shooterPivot.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("PivotPos", shooterPivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("ElevatorPos", elevatorMotor.getPosition().getValueAsDouble()); //up 0.015 down .13
  }

  public void stopPivot() {
    shooterPivot.set(0);
  }

  public void setPivotPos(double pos) {
      shooterPivot.setControl(m_MotionMagicVoltage.withPosition(pos));
  }

  public void setElevatorPos(double pos) {
    elevatorMotor.setControl(m_MotionMagicVoltage.withPosition(pos).withSlot(0));
    shooterPivot.setControl(m_MotionMagicVoltage.withPosition(.12).withSlot(0));
  }

  public void stopElevator() {
    elevatorMotor.set(0);
  }

  public void setPivotSpeed(double velo) {
    shooterPivot.setControl(m_voltageVelocity.withVelocity(velo));
  }

  public void setElevateSpeed(double speed) {
    elevatorMotor.setControl(m_voltageVelocity.withVelocity(speed));
    //shooterPivot.setControl(m_MotionMagicVoltage.withPosition(.04).withSlot(0));
  }
  
  private void setPivotWithLimelight(){
    double target = LimelightHelpers.getTY("limelight-april");

    double angle = map.getInterpolated(new InterpolatingDouble(target)).value;
    setPivotPos(angle);
  }

  public double getPivotAngle(){
    return shooterPivot.getPosition().getValueAsDouble();
  }

  public Command pivotCommand() {
    return this.runEnd(() -> {setPivotSpeed(-.1);}, () -> stopPivot());
  }

  public Command pivotBackCommand() {
    return this.runEnd(() -> setPivotSpeed(.1), () -> stopPivot());
  }

  public Command ElevateCommand() {
    return this.runEnd(() -> setElevateSpeed(1), () -> stopElevator());
  }

  public Command DeElevateCommand() {
    return this.runEnd(() -> setElevateSpeed(-1), () -> stopElevator());
  }

  public Command pivotToPosCommand(double pos) {
    return this.run(() -> setPivotPos(pos));
  }

  public Command elevateToPosCommand(double pos) {
    return this.run(() -> setElevatorPos(pos));
  }

  public Command setShooterPivotWithLimelight(){
    return this.runEnd(() -> {
      state = AimingState.kAiming;
      double target = LimelightHelpers.getTY("limelight-april");
      Boolean valid = LimelightHelpers.getTV("limelight-april");

      double angle = map.getInterpolated(new InterpolatingDouble(target)).value;


      // if (DriverStation.getAlliance().isPresent()) {
      //   if (DriverStation.getAlliance().get() == Alliance.Red){
      //       angle -= .01;
      //   }
      // }
      

      if (valid) {
        setPivotPos(angle);
      }

      if ((Math.abs(getPivotAngle() - angle) < .0025) && valid) {
        state = AimingState.kAimed;
      }

      if (target < -15.67 || target > 22.76) {
        state = AimingState.kTooFar;
      }
    }, () -> {
      stopPivot();
      state = AimingState.kNotAiming;
    });
  }

  public Command moveToAmpPosition(){
    return this. run(() -> {
      setPivotPos(.12);
      setElevatorPos(2);
    });
  }

  public Command moveToHumanPosition(){
    return this. run(() -> {
      setPivotPos(.12);
      setElevatorPos(1.75);
    });
  }

  public Command returnHome(){
    return elevateToPosCommand(0);
  }

  public AimingState getState(){
    return state;
  }
}
