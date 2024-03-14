// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command get3PieceAuto(Drivetrain drivetrain, Intake intake, Shooter shooter, Superstructure superstructure){
    drivetrain.setLocation(0.96, 4.4, -60);
    return 
          shooter.runShooter(90).raceWith(Commands.waitSeconds(0.5))
        .andThen(
          superstructure.pivotToPosCommand(.12).raceWith(Commands.waitSeconds(0.5)))
        .andThen(
            intake.feedCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())))
        .andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)))
        .andThen(
            drivetrain.FollowPath("RightNoteRace").raceWith(intake.intakeCommand()))
        .andThen(
            intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor())))
        .andThen(
            superstructure.setShooterWithLimelight().alongWith(drivetrain.alignCommand())
                .raceWith(Commands.waitSeconds(.25)))
        .andThen(
            intake.feedCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())))
        .andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)))
        .andThen(
            drivetrain.FollowPath("StageMidNote").raceWith(intake.intakeCommand()))
        .andThen(
            intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor())))
        .andThen(
            superstructure
                .setShooterWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)))
        .andThen(
            intake.feedCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())))
        .andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)));
  }

  public static Command get5NoteAuto(Drivetrain drivetrain, Intake intake, Shooter shooter, Superstructure superstructure){
    drivetrain.setLocation(1.42, 5.55, 0);
    return
          shooter.runShooter(90).raceWith(Commands.waitSeconds(0.5))
        .andThen(
          superstructure.pivotToPosCommand(.12).raceWith(Commands.waitSeconds(0.5)))
        .andThen(
            intake.feedCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())))
        .andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)))
        .andThen(
            drivetrain.FollowPath("CenterFront").raceWith(intake.intakeCommand()))
        .andThen(
            intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor())))
        .andThen(
            superstructure.setShooterWithLimelight().alongWith(drivetrain.alignCommand())
                .raceWith(Commands.waitSeconds(.25)))
        .andThen(
            intake.feedCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())))
        .andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)))
        .andThen(
            drivetrain.FollowPath("CenterToRight").raceWith(intake.intakeCommand()))
        .andThen(
            intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor())))
        .andThen(
            superstructure
                .setShooterWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)))
        .andThen(
            intake.feedCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())))
        .andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)))
        .andThen(
            drivetrain.FollowPath("RightToLeft").raceWith(intake.intakeCommand()))
        .andThen(
            intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor())))
        .andThen(
            superstructure.setShooterWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)))
        .andThen(
            intake.feedCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())))
        .andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)))
        .andThen(  
            drivetrain.FollowPath("New New Path").raceWith(intake.intakeCommand()))
        .andThen(
            intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor())))
        .andThen(
            superstructure.setShooterWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)))
        .andThen(
            intake.feedCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())));
  }
}
