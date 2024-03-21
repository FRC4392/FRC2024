// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.deceivers.swerve;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

/** Add your docs here. */
public class SwerveConfigs {
    // Configure drive motor controller parameters
    // Configure current limits
    public static CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentThreshold(80)
        .withSupplyTimeThreshold(1.275)
        .withSupplyCurrentLimitEnable(true)

        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

        // Configure drive control
    public static MotorOutputConfigs driveOutputConfigs = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

        // Configure sensor feedback
    public static FeedbackConfigs driveFeedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio((0.2393893602 / ((20.0 / 13.0) * (45.0 / 15.0))) / 60);

        // Configure PID
    public static Slot0Configs driveSlotConfigs = new Slot0Configs()
        .withKV(0)
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withKS(0);

        // Configure beeps
    public static AudioConfigs driveAudioConfigs = new AudioConfigs()
        .withAllowMusicDurDisable(true)
        .withBeepOnBoot(true)
        .withBeepOnConfig(true);

        // Apply Drive motor configuration
    public static TalonFXConfiguration driveConfigs = new TalonFXConfiguration()
        .withAudio(driveAudioConfigs)
        .withCurrentLimits(driveCurrentLimits)
        .withMotorOutput(driveOutputConfigs)
        .withFeedback(driveFeedbackConfigs)
        .withSlot0(driveSlotConfigs);

    //Azimuth Configs
    //Azimuth rotation inverted?
    public static boolean azimuthIsInverted = true;

    //Brake or coast mode
    public static IdleMode azimuthIdleMode = IdleMode.kBrake;

    //azimuth current limit
    public static int azimuthCurrentLimit = 20;

    // Azimuith absolute encoder position conversion factor
    public static double azimuthAbsolutePositionConversionFactor = 360;

    // Azimuth absolute encoder inverted
    public static boolean azimuthAbsoluteInverted = true;

    // Azimuth absolute encoder sampling depth
    public static int azimuthAbsoluteSamplingDepth = 1;

    // Azimuth incremental encoder conversion factor
    //rev planetary 5:1 = 5.23, 4:1 = 3.61
    //58t big pulley, 18t small pulley
    public static double azimuthIncrementalPositionConversionFactor = 360.0 / (5.23 * 3.61 * (58.0 / 18.0));

    // kp
    public static double azimuthPIDkP = 0.05;

    //does azimuth pid wrap
    public static boolean azimuthPIDwrapping = true;

    // azimuth pid min wrap
    public static double azimuthPIDMinWrap = 0;

    // azimuth pid max wrap
    public static double azimuthPIDMaxWrap = 360;


}
