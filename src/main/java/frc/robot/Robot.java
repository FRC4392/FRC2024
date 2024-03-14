// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Uppies;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
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

  private Drivetrain drivetrain = new Drivetrain();
  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private Uppies climber = new Uppies();
  private Superstructure superstructure = new Superstructure();
  private LED led = new LED();

  SendableChooser<Command> autChooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    led.setShooterLED(0, 0, 1);
    led.setElevatorLED(0, 0, 255);
    configureButtonBindings();
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
  public void disabledPeriodic() {
    SmartDashboard.putString("Selected Auto", autChooser.getSelected().getName());
  }

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

    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverController));

    BooleanSupplier sensorSupplier = () -> intake.getShooterSensor();
    Trigger shooterOccupied = new Trigger(sensorSupplier);
    shooterOccupied.whileTrue(led.setLedsOccupied());

    DoubleSupplier shotSpeed = () -> operatorController.getLeftTriggerAxis();
    DoubleSupplier wallSpeed = () -> operatorController.getRightY();
    Trigger AltButton = operatorController.rightStick();
    Trigger FixShotClose = operatorController.a();
    Trigger HumanTake = operatorController.y();
    Trigger OpOuttake = operatorController.x();
    Trigger OpIntake = operatorController.b();
    Trigger ClimbUp = operatorController.povUp();
    Trigger ClimbDown = operatorController.povDown();
    Trigger Feed = operatorController.rightTrigger(0);
    Trigger Shoot = operatorController.leftStick().and(AltButton.negate());
    Trigger SlowShoot = operatorController.leftStick().and(AltButton);
    Trigger PivotUp = operatorController.leftBumper();
    Trigger PivotDown = operatorController.rightBumper();
    Trigger AutoAim = operatorController.leftTrigger(.1);

    AutoAim.whileTrue(superstructure.setShooterWithLimelight());
    FixShotClose.whileTrue(superstructure.pivotToPosCommand(.12));
    HumanTake.whileTrue(shooter.humanTakeCommand().alongWith(intake.outfeedCommand()).alongWith(superstructure.pivotToPosCommand(.12)));
    Feed.whileTrue(intake.feedCommand());
    PivotUp.whileTrue(superstructure.pivotCommand());
    PivotDown.whileTrue(superstructure.pivotBackCommand());
    Shoot.whileTrue(shooter.runShooter(90));
    SlowShoot.whileTrue(shooter.runShooter(130));
    OpIntake.and(shooterOccupied).whileTrue(intake.intakeCommand());
    OpOuttake.whileTrue(intake.outfeedCommand());
    ClimbUp.whileTrue(climber.ClimbUpCommand());
    ClimbDown.whileTrue(climber.ClimbDownCommand());
    AltButton.whileTrue(led.setLedsPurple().alongWith(climber.ClimbCommand(wallSpeed)));

    operatorController.povLeft().whileTrue(superstructure.ElevateCommand());
    operatorController.povRight().whileTrue(superstructure.DeElevateCommand());

    //operatorController.povLeft().whileTrue(superstructure.pivotToPosCommand(.0335));
    //operatorController.a().whileFalse(shooter.ElevateCommand(elevateSpeed));

    BooleanSupplier brakeSupplier = () -> driverController.getXButton();
    BooleanSupplier intakeButton = () -> driverController.getLeftStickButton();
    BooleanSupplier outtakeButton = () -> driverController.getRightBumper();
    BooleanSupplier spitButton = () -> driverController.getLeftBumper();

    Trigger brake = new Trigger(brakeSupplier);
    Trigger driverIntake = new Trigger(intakeButton);
    Trigger driverOuttake = new Trigger(outtakeButton);
    Trigger driverSpit = new Trigger(spitButton);

    brake.whileTrue(drivetrain.brakeCommand());
    driverIntake.and(shooterOccupied).whileTrue(intake.intakeCommand().alongWith(superstructure.pivotToPosCommand(0)));
    driverOuttake.whileTrue(intake.outtakeCommand());
    driverSpit.whileTrue(shooter.spitCommand().alongWith(intake.intakeCommand()));
  }

  public void ConfigureAutonomousMode(){

    Consumer<Command> autoConsumer = command -> {
      m_autonomousCommand = autChooser.getSelected();
    };

    autChooser.setDefaultOption("5 Note", Autos.get5NoteAuto(drivetrain, intake, shooter, superstructure));
    autChooser.addOption("Right 3 Piece", Autos.get3PieceAuto(drivetrain, intake, shooter, superstructure));
    autChooser.onChange(autoConsumer);
  }


}
