// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.DriverStation;
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

    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()){
    if (alliance.get() == DriverStation.Alliance.Red){
        return (drivetrain.setLocationCommand(16.54-.96, 4.4, -120).andThen(shooter.runShooter(90).raceWith(Commands.waitSeconds(0.5)))).andThen(
        superstructure.pivotToPosCommand(.12).raceWith(Commands.waitSeconds(0.5))).andThen(
          intake.feedFastCommand()).raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01))).andThen(
              drivetrain.FollowPath("RightNoteRace").raceWith(intake.intakeCommand())).andThen(
                intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                  intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
                    superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(
                    drivetrain.FollowPath("StageMidNote").raceWith(intake.intakeCommand()))).andThen(
                      intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                        superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                          intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))).andThen(
                          superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01))))));
    } else {
        return (drivetrain.setLocationCommand(.96, 4.4, -60).andThen(shooter.runShooter(90).raceWith(Commands.waitSeconds(0.5)))).andThen(
        superstructure.pivotToPosCommand(.12).raceWith(Commands.waitSeconds(0.5))).andThen(
          intake.feedFastCommand()).raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01))).andThen(
              drivetrain.FollowPath("RightNoteRace").raceWith(intake.intakeCommand())).andThen(
                intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                  intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
                    superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(
                    drivetrain.FollowPath("StageMidNote").raceWith(intake.intakeCommand()))).andThen(
                      intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                        superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                          intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))).andThen(
                          superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01))))));
    }
} else {
        return (drivetrain.setLocationCommand(16.54-.96, 4.4, -120).andThen(shooter.runShooter(90).raceWith(Commands.waitSeconds(0.5)))).andThen(
        superstructure.pivotToPosCommand(.12).raceWith(Commands.waitSeconds(0.5))).andThen(
          intake.feedFastCommand()).raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01))).andThen(
              drivetrain.FollowPath("RightNoteRace").raceWith(intake.intakeCommand())).andThen(
                intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                  intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
                    superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(
                    drivetrain.FollowPath("StageMidNote").raceWith(intake.intakeCommand()))).andThen(
                      intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                        superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                          intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))).andThen(
                          superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01))))));
}
    

  }

  public static Command get5NoteAuto(Drivetrain drivetrain, Intake intake, Shooter shooter, Superstructure superstructure){
    
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()){
    if (alliance.get() == DriverStation.Alliance.Red){
        return (drivetrain.setLocationCommand(16.54-1.42, 5.55, -180).andThen(shooter.runShooter(90).raceWith(Commands.waitSeconds(0.5)))).andThen(
        superstructure.pivotToPosCommand(.12).raceWith(Commands.waitSeconds(0.5))).andThen(
          intake.feedFastCommand()).raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01))).andThen(
              drivetrain.FollowPath("CenterFront").raceWith(intake.intakeCommand())).andThen(
                intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                  intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
                    superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(
                    drivetrain.FollowPath("CenterToRight").raceWith(intake.intakeCommand()))).andThen(
                      intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                        superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                          intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))).andThen(
                          superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(  
                            drivetrain.FollowPath("RightToLeft").raceWith(intake.intakeCommand()))).andThen(
                              intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                                superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                                  intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))).andThen(
                                     superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(  
                                      drivetrain.FollowPath("LeftMidRave").raceWith(intake.intakeCommand()))).andThen(
                                        intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                                          superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                                            intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))))))));
    } else {
        return (drivetrain.setLocationCommand(1.42, 5.55, 0).andThen(shooter.runShooter(90).raceWith(Commands.waitSeconds(0.5)))).andThen(
        superstructure.pivotToPosCommand(.12).raceWith(Commands.waitSeconds(0.5))).andThen(
          intake.feedFastCommand()).raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01))).andThen(
              drivetrain.FollowPath("CenterFront").raceWith(intake.intakeCommand())).andThen(
                intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                  intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
                    superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(
                    drivetrain.FollowPath("CenterToRight").raceWith(intake.intakeCommand()))).andThen(
                      intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                        superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                          intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))).andThen(
                          superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(  
                            drivetrain.FollowPath("RightToLeft").raceWith(intake.intakeCommand()))).andThen(
                              intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                                superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                                  intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))).andThen(
                                     superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(  
                                      drivetrain.FollowPath("LeftMidRave").raceWith(intake.intakeCommand()))).andThen(
                                        intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                                          superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                                            intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))))))));
    }
} else {
    return (drivetrain.setLocationCommand(16.54-1.42, 5.55, 0+180).andThen(shooter.runShooter(90).raceWith(Commands.waitSeconds(0.5)))).andThen(
        superstructure.pivotToPosCommand(.12).raceWith(Commands.waitSeconds(0.5))).andThen(
          intake.feedFastCommand()).raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
            superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01))).andThen(
              drivetrain.FollowPath("CenterFront").raceWith(intake.intakeCommand())).andThen(
                intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                  intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor())).andThen(
                    superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(
                    drivetrain.FollowPath("CenterToRight").raceWith(intake.intakeCommand()))).andThen(
                      intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                        superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                          intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))).andThen(
                          superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(  
                            drivetrain.FollowPath("RightToLeft").raceWith(intake.intakeCommand()))).andThen(
                              intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                                superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                                  intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))).andThen(
                                     superstructure.pivotToPosCommand(0).raceWith(Commands.waitSeconds(0.01)).andThen(  
                                      drivetrain.FollowPath("LeftMidRave").raceWith(intake.intakeCommand()))).andThen(
                                        intake.intakeCommand().raceWith(Commands.waitUntil(() -> !intake.getShooterSensor()))).andThen(
                                          superstructure.setShooterPivotWithLimelight().alongWith(drivetrain.alignCommand()).raceWith(Commands.waitSeconds(.25)).andThen(
                                            intake.feedFastCommand().raceWith(Commands.waitUntil(() -> intake.getShooterSensor()))))))));
}
  }
}
