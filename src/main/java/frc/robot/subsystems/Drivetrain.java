// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.deceivers.drivers.LimelightHelpers;
import org.deceivers.swerve.SwerveDrive;
import org.deceivers.swerve.SwerveModuleV3;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

    private final CANSparkFlex mDriveMotor1 = new CANSparkFlex(11, MotorType.kBrushless);
    private final CANSparkFlex mDriveMotor2 = new CANSparkFlex(13, MotorType.kBrushless);
    private final CANSparkFlex mDriveMotor3 = new CANSparkFlex(15, MotorType.kBrushless);
    private final CANSparkFlex mDriveMotor4 = new CANSparkFlex(17, MotorType.kBrushless);

    private final CANSparkMax mAzimuth1 = new CANSparkMax(12, MotorType.kBrushless);
    private final CANSparkMax mAzimuth2 = new CANSparkMax(14, MotorType.kBrushless);
    private final CANSparkMax mAzimuth3 = new CANSparkMax(16, MotorType.kBrushless);
    private final CANSparkMax mAzimuth4 = new CANSparkMax(18, MotorType.kBrushless);

    private final Pigeon2 pidgey = new Pigeon2(10);

    private final SwerveModuleV3 Module1 = new SwerveModuleV3(mAzimuth1, mDriveMotor1, new Translation2d(0.256519, 0.256519), "Module 1");
    private final SwerveModuleV3 Module2 = new SwerveModuleV3(mAzimuth2, mDriveMotor2, new Translation2d(0.256519, -0.256519), "Module 2");
    private final SwerveModuleV3 Module3 = new SwerveModuleV3(mAzimuth3, mDriveMotor3, new Translation2d(-0.256519, -0.256519), "Module 3");
    private final SwerveModuleV3 Module4 = new SwerveModuleV3(mAzimuth4, mDriveMotor4, new Translation2d(-0.256519,  0.256519), "Module 4");

    private final SwerveDrive mSwerveDrive = new SwerveDrive(this::getRotation, Module1, Module2, Module3, Module4);

    private double gyroOffset = 0;

    private PIDController rotationController = new PIDController(.5, 0, 0.00);

    
  public Drivetrain() {
    pidgey.setYaw(0);
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
  }

  public void setLocation(double x, double y, double angle){
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

}
