/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.deceivers.drivers.LimelightHelpers;
import org.deceivers.util.JoystickHelper;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

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
  private PIDController rotationController = new PIDController(0.4, .0, .0);

  private PIDController limController = new PIDController(0.3, 0.0, 0.0);
  private PIDController strafeController = new PIDController(0.3,0,0);

  private SlewRateLimiter xfilter = new SlewRateLimiter(1000);
  private SlewRateLimiter yfilter = new SlewRateLimiter(1000);
  private SlewRateLimiter rotfilter = new SlewRateLimiter(1000);

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
    if (mController.getRightBumper()) {
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

    rotVel = mController.getLeftTriggerAxis() - mController.getRightTriggerAxis();
    yVel = yHelper.setInput(yVel).applyPower(2).value;
    xVel = xHelper.setInput(xVel).applyPower(2).value;
    rotVel = rotHelper.setInput(rotVel).applyPower(2).value;

    yrVel = yrHelper.setInput(yrVel).applyPower(yrVel).value;
    xrVel = xrHelper.setInput(xrVel).applyPower(yrVel).value;

    yVel = yVel * driveFactor;
    xVel = xVel * driveFactor;
    rotVel = rotVel * driveFactor;

    Rotation2d joystickAngle = Rotation2d.fromRadians(Math.atan2(-mController.getRightX(), -mController.getRightY()));
    if (!mController.getLeftBumper()) {
      joystickAngle = joystickAngle.plus(Rotation2d.fromDegrees(180));
    }

    double joystickMagnitude = Math.sqrt(
        (mController.getRightY() * mController.getRightY()) + (mController.getRightX() * mController.getRightX()));
    if (joystickMagnitude > .1) {
      rotVel = -rotationController.calculate(Rotation2d.fromDegrees(mDrivetrain.getRotation()).getRadians(),
          joystickAngle.getRadians());
      if (Math.abs(rotVel) > joystickMagnitude) {
        rotVel = joystickMagnitude * Math.signum(rotVel);
      }
    }

    rotVel = -rotVel; // controls were inverted

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

    boolean fieldRelative = !mController.getAButton();
        
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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
