// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.deceivers.swerve.SwerveDrive;
import org.deceivers.swerve.SwerveModuleV3;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Uppies;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LED;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private XboxController driverController = new XboxController(0);
  private CommandXboxController operatorController = new CommandXboxController(1);
  private CommandXboxController testController = new CommandXboxController(2);


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

  private final SwerveDrive mSwerveDrive = new SwerveDrive(()->pidgey.getYaw().getValueAsDouble(), Module1, Module2, Module3, Module4);


  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private Uppies climber = new Uppies();
  LED led = new LED();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    led.setLEDColor(0, 0, 100);
    configureButtonBindings();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public void configureButtonBindings(){

    BooleanSupplier sensorSupplier = () -> shooter.getShooterSensor();
    Trigger shooterOccupied = new Trigger(sensorSupplier);
    shooterOccupied.whileTrue(led.setLedOccupied());

    //DoubleSupplier shotSpeed = () -> operatorController.getLeftTriggerAxis();
    DoubleSupplier wallSpeed = () -> operatorController.getRightY();
    DoubleSupplier elevateSpeed = () -> operatorController.getLeftY();
    Trigger HighShot = operatorController.a();
    Trigger LowShot = operatorController.x();
    Trigger HumanTake = operatorController.y();
    Trigger ClimbUp = operatorController.povUp();
    Trigger ClimbDown = operatorController.povDown();
    Trigger Feed = operatorController.b();
    Trigger PivotUp = operatorController.leftBumper();
    Trigger PivotDown = operatorController.rightBumper();

    HighShot.whileTrue(shooter.pivotToPosCommand(.06));
    LowShot.whileTrue(shooter.pivotToPosCommand(.085));
    HumanTake.whileTrue(shooter.backfeedCommand());
    Feed.whileTrue(shooter.feedCommand());
    PivotUp.whileTrue(shooter.pivotCommand());
    PivotDown.whileTrue(shooter.pivotBackCommand());
    operatorController.leftTrigger(0).whileTrue(shooter.runShooter());
    ClimbUp.whileTrue(climber.ClimbUpCommand());
    ClimbDown.whileTrue(climber.ClimbDownCommand());
    operatorController.a().whileFalse(climber.WallDriveCommand(wallSpeed));
    //operatorController.a().whileFalse(shooter.ElevateCommand(elevateSpeed));

    testController.povUp().whileTrue(shooter.ElevateCommand());
    testController.povDown().whileTrue(shooter.DeElevateCommand());
    testController.leftBumper().whileTrue(shooter.CalibrateKs(1));
    testController.rightBumper().whileTrue(shooter.CalibrateKs(-1));
    testController.y().whileTrue(shooter.pivotToPosCommand(.08));
    testController.b().whileTrue(shooter.pivotToPosCommand(.12));
    testController.x().whileTrue(shooter.elevateToPosCommand(1));
    testController.a().whileTrue(shooter.elevateToPosCommand(0));

    BooleanSupplier brakeSupplier = () -> driverController.getXButton();
    BooleanSupplier intakeButton = () -> driverController.getLeftStickButton();
    BooleanSupplier outtakeButton = () -> driverController.getRightBumper();
    BooleanSupplier spitButton = () -> driverController.getLeftBumper();

    Trigger brake = new Trigger(brakeSupplier);
    Trigger driverIntake = new Trigger(intakeButton);
    Trigger driverOuttake = new Trigger(outtakeButton);
    Trigger driverSpit = new Trigger(spitButton);

    brake.whileTrue(mSwerveDrive.brakeCommand());
    driverIntake.and(shooterOccupied).whileTrue(intake.intakeCommand().alongWith(shooter.feedWithPosCommand()));
    driverOuttake.whileTrue(intake.outtakeCommand().alongWith(shooter.outfeedCommand()));
    driverSpit.whileTrue(shooter.spitCommand().alongWith(intake.intakeCommand()));


  }
}
