/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.deceivers.drivers.LimelightHelpers;
import org.deceivers.util.JoystickHelper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.AimingState;

public class DriveCommand extends CommandBase {
  public final Drivetrain mDrivetrain;
  public XboxController mController;
  public CommandXboxController opController;
  private boolean lastScan;

  private JoystickHelper xHelper = new JoystickHelper(0);
  private JoystickHelper yHelper = new JoystickHelper(0);
  private JoystickHelper rotHelper = new JoystickHelper(0);
  private JoystickHelper xrHelper = new JoystickHelper(0);
  private JoystickHelper yrHelper = new JoystickHelper(0);
  private double driveFactor = 1;
  private PIDController rotationController = new PIDController(.7, 0, 0.00);

  private PIDController limController = new PIDController(0.01, 0.0, 0.0);
  private PIDController strafeController = new PIDController(0.01,0,0);

  private SlewRateLimiter xfilter = new SlewRateLimiter(1000);
  private SlewRateLimiter yfilter = new SlewRateLimiter(1000);
  private SlewRateLimiter rotfilter = new SlewRateLimiter(1000);
  private double lastSpeed; 
  private double lastTime;

  Rotation2d angle = new Rotation2d(0);

  public DriveCommand(Drivetrain Drivetrain, XboxController XboxController) {
    mDrivetrain = Drivetrain;
    mController = XboxController;
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    limController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(mDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = 0;
    double yVel = 0;
    double rotVel = 0;
    double xrVel = 0;
    double yrVel = 0;

    yVel = mController.getLeftY();
    xVel = mController.getLeftX();

    yrVel = mController.getRightY();
    xrVel = mController.getRightX();

    // slow down `on
    if (mController.getRightStickButton()) {
      driveFactor = 0.5;
    } else {
      driveFactor = 1.0;
    }

    if (DriverStation.isTeleop() && (DriverStation.getMatchTime() < 40.0) && (DriverStation.getMatchTime() > 39.0)) {
      mController.setRumble(RumbleType.kLeftRumble, 1);
      mController.setRumble(RumbleType.kRightRumble, 1);
    } else {
      mController.setRumble(RumbleType.kLeftRumble, 0);
      mController.setRumble(RumbleType.kRightRumble, 0);
    }

    rotVel = -mController.getRightX();
    yVel = yHelper.setInput(yVel).applyPower(2).value;
    xVel = xHelper.setInput(xVel).applyPower(2).value;
    rotVel = rotHelper.setInput(rotVel).applyPower(2).value;

    yrVel = yrHelper.setInput(yrVel).applyPower(yrVel).value;
    xrVel = xrHelper.setInput(xrVel).applyPower(yrVel).value;

    
    if (mController.getRightTriggerAxis() > .1){
        new Rotation2d();
        var targetValid = LimelightHelpers.getTV("limelight-april");
        angle = Rotation2d.fromDegrees(-LimelightHelpers.getTX("limelight-april")).plus(Rotation2d.fromDegrees(mDrivetrain.getRotation()));
        
        if (targetValid) {
          rotVel = rotationController.calculate(Rotation2d.fromDegrees(mDrivetrain.getRotation()).getRadians(), angle.getRadians());
        }



      if (Math.abs(LimelightHelpers.getTX("limelight-april")) < 1.5 && targetValid) {
        mDrivetrain.setState(AimingState.kAimed);
      } else {
        mDrivetrain.setState(AimingState.kAiming);
      }

      SmartDashboard.putNumber("DrivetrainTargetAngle", angle.getDegrees());

    } else if (mController.getBButton()) {
      // new Rotation2d();
      // angle = Rotation2d.fromDegrees(0);
      // rotVel = rotationController.calculate(Rotation2d.fromDegrees(mDrivetrain.getRotation()).getRadians(), angle.getRadians());
    } else {
      rotationController.reset();
      mDrivetrain.setState(AimingState.kNotAiming);
    }

    yVel = yVel * driveFactor;
    xVel = xVel * driveFactor;
    rotVel = rotVel * driveFactor;

    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    //   if (alliance.get() == Alliance.Red) {
    //     yVel *= -1;
    //     xVel *= -1;
    //   }
    // }

    // Rotation2d joystickAngle = Rotation2d.fromRadians(Math.atan2(-mController.getRightX(), -mController.getRightY()));
    // if (!mController.getLeftBumper()) {
    //   joystickAngle = joystickAngle.plus(Rotation2d.fromDegrees(180));
    // }

    // double joystickMagnitude = Math.sqrt(
    //     (mController.getRightY() * mController.getRightY()) + (mController.getRightX() * mController.getRightX()));
    // if (joystickMagnitude > .1) {
    //   rotVel = -rotationController.calculate(Rotation2d.fromDegrees(mDrivetrain.getRotation()).getRadians(),
    //       joystickAngle.getRadians());
    //   if (Math.abs(rotVel) > joystickMagnitude) {
    //     rotVel = joystickMagnitude * Math.signum(rotVel);
    //   }
    //}

    
    if (mController.getRawButton(7) & !lastScan) {
      mDrivetrain.resetGyro();
    }
    lastScan = mController.getRawButton(7);

    // boolean fieldRelative = !(mController.getRightTriggerAxis()>0);
    // boolean fieldRelative = true;

    // if (fieldRelative){
    // mDrivetrain.drive(yfilter.calculate(yVel), xfilter.calculate(xVel),
    // rotfilter.calculate(rotVel), fieldRelative);
    // } else {
    // mDrivetrain.drive(yVel*-1, xVel*-1, rotVel, fieldRelative);
    // }

    boolean fieldRelative = !mController.getYButton();

    if (mController.getLeftTriggerAxis() > 0.1){
      fieldRelative = false;

      new Rotation2d();
        var targetValid = LimelightHelpers.getTV("limelight-note");
        angle = Rotation2d.fromDegrees(-LimelightHelpers.getTX("limelight-note")).plus(Rotation2d.fromDegrees(mDrivetrain.getRotation()));
        
        if (targetValid) {
          rotVel = rotationController.calculate(Rotation2d.fromDegrees(mDrivetrain.getRotation()).getRadians(), angle.getRadians());
        }



      if (Math.abs(LimelightHelpers.getTX("limelight-april")) < 1.5 && targetValid) {
        mDrivetrain.setState(AimingState.kAimed);
      } else {
        mDrivetrain.setState(AimingState.kAiming);
      }

      SmartDashboard.putNumber("DrivetrainTargetAngle", angle.getDegrees());
    } else if (mController.getBButton()){

      var tempXVel = xVel;
      var tempYVel = yVel;

      rotVel = rotationController.calculate(Rotation2d.fromDegrees(mDrivetrain.getRotation()).getRadians(), Rotation2d.fromDegrees(-90).getRadians());

      var ampValid = LimelightHelpers.getTV("limelight-note");
      var error = LimelightHelpers.getTX("limelight-note");

      if (ampValid) {
      xVel = strafeController.calculate(error, 0);
      } else {
        strafeController.reset();
        xVel = tempYVel;

        var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        xVel *= -1;
      }
    }
      }

      yVel = -tempXVel;

      var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        yVel *= -1;
      }
    }

      fieldRelative = false;

    } else if (mController.getAButton()) {
      rotVel = rotationController.calculate(Rotation2d.fromDegrees(mDrivetrain.getRotation()).getRadians(), Rotation2d.fromDegrees(90).getRadians());
    }

     var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red && fieldRelative) {
        yVel *= -1;
        xVel *= -1;
      }
    }

    

    
        
        //X direction is arm direction on robot, y direction is out from the apriltag
        mDrivetrain.drive(yfilter.calculate(yVel), xfilter.calculate(xVel), rotfilter.calculate(rotVel), fieldRelative); 

    // mDrivetrain.drive(yVel,xVel, rotVel, fieldRelative);
    // mDrivetrain.setModulesAngle(xVel);
  }


  // Called once the command ends or is interrupted.
 @Override
  public void end(boolean interrupted) {
    mDrivetrain.stop();
  }

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
