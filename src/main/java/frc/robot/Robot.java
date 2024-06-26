// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Alert.AlertType;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
//import frc.robot.subsystems.Uppies;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Uppies;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.OccupancyState;
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

  public boolean drivemode;

  private Drivetrain drivetrain = new Drivetrain();
  private Intake intake = new Intake();
  private Shooter shooter = new Shooter();
  private Uppies climber = new Uppies();
  private Superstructure superstructure = new Superstructure();
  private PowerDistribution m_pdp = new PowerDistribution();

  Supplier<OccupancyState> occupySupplier = () -> intake.getOccupancy();
  Supplier<IntakeState> intakeSupplier = () -> intake.getState();
  Supplier<Superstructure.AimingState> shootSupplier = ()->superstructure.getState();
  Supplier<Drivetrain.AimingState> driveShotSupplier = () -> drivetrain.getState();

  private LED led = new LED(intakeSupplier, occupySupplier, shootSupplier, driveShotSupplier);

  SendableChooser<String> autoChooser = new SendableChooser<>();
  SendableChooser<String> demoChooser = new SendableChooser<>();

  Consumer<String> autoConsumer = auto -> {
    if (auto == "5Note") {
      m_autonomousCommand = Autos.get5NoteAuto(drivetrain, intake, shooter, superstructure);
    } else if (auto == "3Piece"){
      m_autonomousCommand = Autos.get3PieceAuto(drivetrain, intake, shooter, superstructure);
    } else if (auto == "Amp") {
      m_autonomousCommand = Autos.getAmpAuto(drivetrain, intake, shooter, superstructure);
    } else if (auto == "MidRush") {
      m_autonomousCommand = Autos.get5SprintNoteAuto(drivetrain, intake, shooter, superstructure);
    }
  };

  Consumer<String> demoConsumer = mode -> {
    if (mode == "comp") {
      drivemode = false;
    } else if (mode == "demo"){
      drivemode = true;
    }
  };

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    led.setElevatorLED(0, 0, 255);

    configureButtonBindings();
    ConfigureAutonomousMode();
    ConfigureDriveMode();

    RobotController.setBrownoutVoltage(6);
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
    SmartDashboard.putBoolean("Drive Mode",drivemode);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
   //double voltage = m_pdp.getVoltage();
    //new Alert("Voltage is " + voltage, AlertType.INFO).set(true);
    // SmartDashboard.putString("Selected Auto", autoChooser.getSelected().getName());

    // if (autoChooser.getSelected() == "5Note") {
    //   m_autonomousCommand = Autos.get5NoteAuto(drivetrain, intake, shooter, superstructure);
    // } else if (autoChooser.getSelected() == "3Piece"){
    //   m_autonomousCommand = Autos.get3PieceAuto(drivetrain, intake, shooter, superstructure);
    // }
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

    // demo buttons
    BooleanSupplier driveModeSupplier = () -> drivemode;
    Trigger demoMode = new Trigger(driveModeSupplier);    
    DoubleSupplier shotSpeed = () -> driverController.getLeftTriggerAxis()*25; 
    BooleanSupplier demoShotButton = () -> driverController.getLeftTriggerAxis() > .1;
    BooleanSupplier demoAmpButton = () -> driverController.getRightBumper();
    BooleanSupplier demoAimButton = () -> driverController.getRightTriggerAxis() > .1;
    BooleanSupplier demoIntakeButton = () -> driverController.getXButton();
    BooleanSupplier demoOuttakeButton = () -> driverController.getBButton();
    BooleanSupplier demoHumanTakeButton = () -> driverController.getLeftBumper();
    BooleanSupplier demoFeedButton = () -> driverController.getYButton();
    BooleanSupplier demoPivotUpButton = () -> driverController.getPOV() == 0;
    BooleanSupplier demoPivotDownButton = () -> driverController.getPOV() == 180;
    BooleanSupplier demoElevatorUpButton = () -> driverController.getPOV() == 270;
    BooleanSupplier demoElevatorDownButton = () -> driverController.getPOV() == 90;

    Trigger demoShot = new Trigger(demoShotButton).and(demoMode);
    Trigger demoAmp = new Trigger(demoAmpButton).and(demoMode);
    Trigger demoAim = new Trigger(demoAimButton).and(demoMode);
    Trigger demoIntake = new Trigger(demoIntakeButton).and(demoMode);
    Trigger demoOuttake = new Trigger(demoOuttakeButton).and(demoMode);
    Trigger demoHumanTake = new Trigger(demoHumanTakeButton).and(demoMode);
    Trigger demoFeed = new Trigger(demoFeedButton).and(demoMode);
    Trigger demoPivotUp = new Trigger(demoPivotUpButton).and(demoMode);
    Trigger demoPivotDown = new Trigger(demoPivotDownButton).and(demoMode);
    Trigger demoElevatorUp = new Trigger(demoElevatorUpButton).and(demoMode);
    Trigger demoElevatorDown = new Trigger(demoElevatorDownButton).and(demoMode);


    //demo specific actions
    demoAim.whileTrue(superstructure.setShooterPivotWithLimelight().alongWith(shooter.runShooterVariable(shotSpeed)));
    demoAim.onFalse(shooter.stopShooter());
    demoShot.whileTrue(shooter.runShooterVariable(shotSpeed));
    demoShot.onFalse(shooter.stopShooter());
    demoAmp.onTrue(superstructure.moveToAmpPosition().alongWith(shooter.softamp()));
    demoAmp.onFalse(superstructure.returnHome().alongWith(shooter.stopShooter()));
    demoHumanTake.whileTrue(shooter.softHumanTakeCommand().alongWith(intake.humantakeCommand()));
    demoHumanTake.onFalse(shooter.stopShooter().alongWith(superstructure.returnHome()));

    DoubleSupplier wallSpeed = () -> operatorController.getRightY();
    Trigger AltButton = operatorController.rightStick();
    Trigger OpSpit = operatorController.rightBumper().and(AltButton.negate());
    Trigger HumanTake = operatorController.y();
    Trigger OpOuttake = operatorController.x();
    Trigger OpIntake = operatorController.b();
    Trigger Feed = operatorController.rightTrigger(0);
    //Trigger Shoot = operatorController.leftStick();
    Trigger reverseFeed = operatorController.a();
    Trigger elevateUp = operatorController.povLeft().and(AltButton);
    Trigger ElevateDown = operatorController.povRight().and(AltButton);

    Trigger FeedShot = operatorController.povDown().and(AltButton.negate());
    Trigger FeedShot2 = operatorController.rightBumper().and(AltButton);
    Trigger FixShotAmp = operatorController.povLeft().and(AltButton.negate());
    Trigger FixShotPodi = operatorController.povRight().and(AltButton.negate());
    Trigger FixShotClose = operatorController.povUp().and(AltButton.negate());

    Trigger PivotUp = operatorController.povUp().and(AltButton);
    Trigger PivotDown = operatorController.povDown().and(AltButton);

    Trigger AutoAim = operatorController.leftTrigger(.1);

    AltButton.whileTrue(climber.ClimbCommand(wallSpeed));

    Trigger intakedTrigger = new Trigger(() -> intake.getEntrySwitch());

    Trigger endGameTrigger = new Trigger(() -> DriverStation.getMatchTime() < 25);

    endGameTrigger.onTrue(Commands.runEnd(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1), () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0), new Subsystem[0]).withTimeout(1));

    intakedTrigger.onTrue(Commands.runEnd(() -> driverController.setRumble(RumbleType.kBothRumble, 1), () -> driverController.setRumble(RumbleType.kBothRumble, 1), new Subsystem[0]).withTimeout(1));

    AutoAim.whileTrue(superstructure.setShooterPivotWithLimelight().alongWith(shooter.runShooter(80)));
    AutoAim.onFalse(shooter.stopShooter());
    FixShotClose.whileTrue(superstructure.pivotToPosCommand(.12).alongWith(shooter.runShooter(80)));
    FixShotClose.onFalse(shooter.stopShooter());
    FixShotAmp.whileTrue(superstructure.pivotToPosCommand(.105).alongWith(shooter.runShooter(80)));
    FixShotAmp.onFalse(shooter.stopShooter());
    FixShotPodi.whileTrue(superstructure.pivotToPosCommand(.11).alongWith(shooter.runShooter(80))); 
    FixShotPodi.onFalse(shooter.stopShooter());
    HumanTake.whileTrue(shooter.humanTakeCommand().alongWith(intake.humantakeCommand()).alongWith(superstructure.moveToHumanPosition()));
    HumanTake.onFalse(shooter.stopShooter().alongWith(superstructure.returnHome()));
    Feed.or(demoFeed).whileTrue(intake.feedCommand());
    reverseFeed.whileTrue(intake.reverseFeedCommand());
    PivotUp.or(demoPivotUp).whileTrue(superstructure.pivotCommand());
    PivotDown.or(demoPivotDown).whileTrue(superstructure.pivotBackCommand());
    //Shoot.whileTrue(shooter.runShooter(shotSpeed.getAsDouble()));
    //Shoot.onFalse(shooter.stopShooter());
    //OpSpit.whileTrue(shooter.spitCommand());

    elevateUp.or(demoElevatorUp).whileTrue(superstructure.ElevateCommand());
    ElevateDown.or(demoElevatorDown).whileTrue(superstructure.DeElevateCommand());

    FeedShot.or(OpSpit).whileTrue(shooter.anythingButFerry().alongWith(superstructure.pivotToPosCommand(0)));
    FeedShot2.whileTrue(shooter.anythingButFerry2().alongWith(superstructure.pivotToPosCommand(.2)));

    operatorController.leftBumper().onTrue(superstructure.moveToAmpPosition().alongWith(shooter.amp()));
    operatorController.leftBumper().onFalse(superstructure.returnHome().alongWith(shooter.stopShooter()));

    //operatorController.povLeft().whileTrue(superstructure.pivotToPosCommand(.0335));
    //operatorController.a().whileFalse(shooter.ElevateCommand(elevateSpeed));

    BooleanSupplier brakeSupplier = () -> driverController.getXButton();
    BooleanSupplier intakeButton = () -> driverController.getLeftStickButton();
    BooleanSupplier outtakeButton = () -> driverController.getRightBumper();
    BooleanSupplier spitButton = () -> driverController.getLeftBumper();


    Trigger driveVoltage = new Trigger(() -> driverController.getAButton());

    
    Trigger brake = new Trigger(brakeSupplier).and(demoMode.negate());
    Trigger driverIntake = new Trigger(intakeButton).and(demoMode.negate());
    Trigger driverOuttake = new Trigger(outtakeButton).and(demoMode.negate());
    Trigger driverSpit = new Trigger(spitButton).and(demoMode.negate());

    brake.whileTrue(drivetrain.brakeCommand());
    OpIntake.or(driverIntake).or(demoIntake).whileTrue(intake.intakeCommand().alongWith(superstructure.pivotToPosCommand(0)));
    OpOuttake.or(driverOuttake).or(demoOuttake).whileTrue(intake.outtakeCommand().alongWith(superstructure.pivotToPosCommand(0)));
    driverSpit.whileTrue(shooter.spitCommand().alongWith(intake.sptiCommand()));

    driveVoltage.whileTrue(drivetrain.driveStraight());

  }

  public void ConfigureAutonomousMode(){

    autoChooser.setDefaultOption("5 Note", "5Note");
    autoChooser.addOption("Source Side Auto", "3Piece");
    autoChooser.addOption("Amp Side Auto", "Amp");
    autoChooser.addOption("Center Mid Rush", "MidRush");

    autoChooser.onChange(autoConsumer);

    SmartDashboard.putData(autoChooser);

    
  }

  public void ConfigureDriveMode(){

    demoChooser.setDefaultOption("Competition", "comp");
    demoChooser.addOption("Demonstration", "demo");

    demoChooser.onChange(demoConsumer);

    SmartDashboard.putData(demoChooser);

  }


}
