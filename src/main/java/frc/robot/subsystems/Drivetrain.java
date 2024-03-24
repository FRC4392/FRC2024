// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.deceivers.drivers.LimelightHelpers;
import org.deceivers.swerve.SwerveDrive;
import org.deceivers.swerve.SwerveModuleV3;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  public enum AimingState {
    kNotAiming,
    kAiming,
    kAimed;
  }

  private final TalonFX mDriveMotor1 = new TalonFX(11);
  private final TalonFX mDriveMotor2 = new TalonFX(13);
  private final TalonFX mDriveMotor3 = new TalonFX(15);
  private final TalonFX mDriveMotor4 = new TalonFX(17);

  private final CANSparkMax mAzimuth1 = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax mAzimuth2 = new CANSparkMax(14, MotorType.kBrushless);
  private final CANSparkMax mAzimuth3 = new CANSparkMax(16, MotorType.kBrushless);
  private final CANSparkMax mAzimuth4 = new CANSparkMax(18, MotorType.kBrushless);

  private final Pigeon2 pidgey = new Pigeon2(10);

  private final SwerveModuleV3 Module1 = new SwerveModuleV3(mAzimuth1, mDriveMotor1,
      new Translation2d(0.256519, 0.256519), "Module 1");
  private final SwerveModuleV3 Module2 = new SwerveModuleV3(mAzimuth2, mDriveMotor2,
      new Translation2d(0.256519, -0.256519), "Module 2");
  private final SwerveModuleV3 Module3 = new SwerveModuleV3(mAzimuth3, mDriveMotor3,
      new Translation2d(-0.256519, -0.256519), "Module 3");
  private final SwerveModuleV3 Module4 = new SwerveModuleV3(mAzimuth4, mDriveMotor4,
      new Translation2d(-0.256519, 0.256519), "Module 4");

  private final SwerveDrive mSwerveDrive = new SwerveDrive(this::getRotation, Module1, Module2, Module3, Module4);

  private double gyroOffset = 0;

  private PIDController rotationController = new PIDController(.5, 0, 0.00);

  Field2d field2d = new Field2d();

  AimingState state = AimingState.kNotAiming;

    
  public Drivetrain() {
    pidgey.setYaw(0);

    SmartDashboard.putData("Swerve Drive", new Sendable() {
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    builder.addDoubleProperty("Front Left Angle", () -> Module1.getAzimuthRotation(), null);
    builder.addDoubleProperty("Front Left Velocity", () -> Module1.getDriveVelocity(), null);

    builder.addDoubleProperty("Front Right Angle", () -> Module2.getAzimuthRotation(), null);
    builder.addDoubleProperty("Front Right Velocity", () -> Module2.getDriveVelocity(), null);

    builder.addDoubleProperty("Back Left Angle", () -> Module3.getAzimuthRotation(), null);
    builder.addDoubleProperty("Back Left Velocity", () -> Module3.getDriveVelocity(), null);

    builder.addDoubleProperty("Back Right Angle", () -> Module4.getAzimuthRotation(), null);
    builder.addDoubleProperty("Back Right Velocity", () -> Module4.getDriveVelocity(), null);

    builder.addDoubleProperty("Robot Angle", () -> getRotation(), null);
  }
});
  }

  public void drive(double forward, double strafe, double azimuth, boolean fieldRelative){
    azimuth = azimuth*2.5;
    mSwerveDrive.drive(forward, strafe, azimuth, fieldRelative);
  }

  public void driveClosedLoop(double forward, double strafe, double azimuth, boolean fieldRelative){
    if (!fieldRelative){
      forward = -forward;
      strafe = -strafe;
    }
    azimuth = azimuth*2.5;
    mSwerveDrive.driveClosedLoop(forward, strafe, azimuth, fieldRelative);
  }

  public void driveAuto(ChassisSpeeds speeds){
    SmartDashboard.putNumber("AutoChasisSpeedsY", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("AutoChasisSpeedsX", speeds.vxMetersPerSecond);
    mSwerveDrive.driveClosedLoop(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  public void driveVoltage(double forward, double strafe, double azimuth, boolean fieldRelative){
    mSwerveDrive.driveVoltage(forward, strafe, azimuth, fieldRelative);
  }

  public void stop(){
    mSwerveDrive.stop();
  }

  public Pose2d getPose(){
    return mSwerveDrive.getPose();
  }

  @Override
  public void periodic() {
    mSwerveDrive.updateOdometry();
    mSwerveDrive.log();

    

    field2d.setRobotPose(getPose());
    SmartDashboard.putData(field2d);

    
  }

  public void setLocation(double x, double y, double angle){
    setGyro(angle);
    mSwerveDrive.setLocation(x, y, angle);
  }

  public ChassisSpeeds getSpeeds(){
      ChassisSpeeds chassisSpeeds = mSwerveDrive.getChassisSpeeds();
      return(chassisSpeeds);
    }

    public void resetGyro(){
      setGyro(0);
    }

    public void setGyro(double position){
      gyroOffset = (position - pidgey.getYaw().getValueAsDouble());
    }
  
    public double getRotation() {
      return Rotation2d.fromRadians(MathUtil.angleModulus(Rotation2d.fromDegrees(pidgey.getYaw().getValueAsDouble() + gyroOffset).getRadians())).getDegrees();
    }
  
    public double getYaw() {
      return pidgey.getYaw().getValueAsDouble();
    }

    public void setModulesAngle(double angle, int module){
      mSwerveDrive.setModulesAngle(angle, module);
    }

    public Command brakeCommand(){
      return this.run(() -> {
        double angle = -45;
        for (int i = 0; i < 4; i++){
          setModulesAngle(angle, i);
          angle += 90;
        }
      });
    }

    public Command balanceCommand(){
      return this.run(() ->{
        double pitch = pidgey.getRoll().getValueAsDouble();
        double maxOutput = .25;
        double output = pitch * .01;

        if (Math.abs(output) > maxOutput){
          output = Math.signum(output) * maxOutput;
        }

        drive(output, 0, 0, false);

        SmartDashboard.putNumber("pitch", pitch);
        SmartDashboard.putNumber("output", output);
      });
    }

    public Command FollowPath(String name){
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);

      return new FollowPathHolonomic(
        path, 
        this::getPose, 
        this::getSpeeds, 
        this::driveAuto, 
        new HolonomicPathFollowerConfig(
          new PIDConstants(5.0, 0.0, 0.0), 
          new PIDConstants(10, 0.0, 0.0),
          4.5,
          .414,
          new ReplanningConfig()
          ),
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);
    }

    public Command alignCommand(){
      return this.runEnd(() -> {
        Rotation2d angle = Rotation2d.fromDegrees(-LimelightHelpers.getTX("limelight-april")).plus(Rotation2d.fromDegrees(this.getRotation()));
        double rotVel = rotationController.calculate(Rotation2d.fromDegrees(this.getRotation()).getRadians(), angle.getRadians());
        drive(0, 0, rotVel, false);
      }, () -> stop());
    }

    public Command setLocationCommand(double x, double y, double rot){
      return this.runOnce(() -> setLocation(x, y, rot));
    }

    public void setState(AimingState state){
      this.state = state; 
    }

    public AimingState getState(){
      return state;
    }

    public Command driveStraight(){
      return this.run(() -> driveVoltage(2.4, 0, 0, false));
    }

    public Command setStraight(){
      return this.run(() -> {
        mSwerveDrive.setModulesAngle(0.0, 0);
        mSwerveDrive.setModulesAngle(0.0, 1);
        mSwerveDrive.setModulesAngle(0.0, 2);
        mSwerveDrive.setModulesAngle(0.0, 3);
      });
    }

}
