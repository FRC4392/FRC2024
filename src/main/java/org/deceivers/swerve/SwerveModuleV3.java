package org.deceivers.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleV3 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final AbsoluteEncoder mAzimuthAbsoluteEncoder;
    private final RelativeEncoder mAzimuthIncrementalEncoder;
    private final SparkPIDController mAzimuthPID;

    private final TalonFX mDriveMotor;
    private final VelocityVoltage mDriveVelocityControl;
    private final StatusSignal<Double> mDrivePositionSignal;
    private final StatusSignal<Double> mDriveVelocitySignal;

    private final Translation2d mLocation;
    private final String mName;

    // need to update the speed to m/s

    public SwerveModuleV3(CANSparkMax azimuthMotor, TalonFX driveMotor,
            Translation2d location, String name) {

        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
        mLocation = location;
        mName = name;

        //Get Drive Controls and Signals
        mDriveVelocityControl = new VelocityVoltage(0).withSlot(0);
        mDrivePositionSignal = mDriveMotor.getPosition();
        mDriveVelocitySignal = mDriveMotor.getVelocity();

        // Get Azimuth Controller and Encoders
        mAzimuthPID = mAzimuthMotor.getPIDController();
        mAzimuthAbsoluteEncoder = mAzimuthMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        mAzimuthIncrementalEncoder = mAzimuthMotor.getEncoder();


        // Configure drive motor controller parameters
        // Configure current limits
        CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentThreshold(80)
        .withSupplyTimeThreshold(1.275)
        .withSupplyCurrentLimitEnable(true)

        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

        // Configure drive control
        MotorOutputConfigs driveOutputConfigs = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

        // Configure sensor feedback
        FeedbackConfigs driveFeedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio((0.2393893602 / ((20.0 / 13.0) * (45.0 / 15.0))) / 60);

        // Configure PID
        Slot0Configs driveSlotConfigs = new Slot0Configs()
        .withKV(0)
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withKS(0);

        // Configure beeps
        AudioConfigs driveAudioConfigs = new AudioConfigs()
        .withAllowMusicDurDisable(true)
        .withBeepOnBoot(true)
        .withBeepOnConfig(true);

        // Apply Drive motor configuration
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.Audio = driveAudioConfigs;
        driveConfigs.CurrentLimits = driveCurrentLimits;
        driveConfigs.MotorOutput = driveOutputConfigs;
        driveConfigs.Feedback = driveFeedbackConfigs;
        driveConfigs.Slot0 = driveSlotConfigs;

        mDriveMotor.getConfigurator().apply(driveConfigs);


        // Configure azimuth motor controller parameters
        // Rest motors to factory defaults to ensure correct parameters
        mAzimuthMotor.restoreFactoryDefaults();

        // Configure azimuth motor controller parameters
        mAzimuthMotor.setInverted(true);
        mAzimuthMotor.setClosedLoopRampRate(0);
        mAzimuthMotor.setOpenLoopRampRate(0);
        mAzimuthMotor.setIdleMode(IdleMode.kBrake);
        mAzimuthMotor.setSmartCurrentLimit(20);

        // Configure drive absolute encoder
        mAzimuthAbsoluteEncoder.setPositionConversionFactor(360);
        mAzimuthAbsoluteEncoder.setInverted(true);
        mAzimuthAbsoluteEncoder.setAverageDepth(1);

        // Configure azimuth incremental encoder
        mAzimuthIncrementalEncoder.setPositionConversionFactor(360.0 / (5.23 * 3.61 * (58.0 / 18.0)));

        // Configure azimuth PID
        mAzimuthPID.setFeedbackDevice(mAzimuthAbsoluteEncoder);
        mAzimuthPID.setP(.05);
        mAzimuthPID.setPositionPIDWrappingEnabled(true);
        mAzimuthPID.setPositionPIDWrappingMinInput(0);
        mAzimuthPID.setPositionPIDWrappingMaxInput(360);

        // Burn flahs in case of power cycle
        mAzimuthMotor.burnFlash();
    }

        // Run when swerve drive is first initialized
    @Override
    public void init() {

    }

    // Sets the drive motor speed in open loop mode
    public void setSpeedOpenLoop(double speed) {
        mDriveMotor.set(speed);
    }

    public void setSpeedClosedLoop(double speed) {
        mDriveMotor.setControl(mDriveVelocityControl.withVelocity(speed));
    }

    // Sets the rotation speed of the azimuth motor in open loop mode
    public void setRotationOpenLoop(double rotation) {
        mAzimuthMotor.set(rotation);
    }

    // set angle of swerve drive
    @Override
    public void setAngle(double angle) {
        mAzimuthPID.setReference(angle, ControlType.kPosition);
    }

    // Set the speed and direction of the swerve module
    @Override
    public void set(SwerveModuleState drive) {
        Rotation2d current = Rotation2d.fromDegrees(mAzimuthAbsoluteEncoder.getPosition());
        SwerveModuleState optimizedState = SwerveModuleState.optimize(drive, current);
        double setpoint = optimizedState.angle.getDegrees();
        double velocity = optimizedState.speedMetersPerSecond;
        mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        mDriveMotor.set(velocity);
    }

    @Override
    public void setClosedLoop(SwerveModuleState drive) {
        Rotation2d current = Rotation2d.fromDegrees(mAzimuthAbsoluteEncoder.getPosition());
        SwerveModuleState optimizedState = SwerveModuleState.optimize(drive, current);
        double setpoint = optimizedState.angle.getDegrees();
        double velocity = optimizedState.speedMetersPerSecond;
        mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        mDriveMotor.setControl(mDriveVelocityControl.withVelocity(velocity));
    }

    // Stop all motors
    @Override
    public void stop() {
        mAzimuthMotor.set(0);
        mDriveMotor.set(0);
    }

    // Gets the speed of the drive motor
    public double getDriveVelocity() {
        return mDriveVelocitySignal.getValueAsDouble();
    }

    // Get the distance of the drive encoder
    public double getDriveDistance() {
        return mDrivePositionSignal.getValueAsDouble();
    }

    // Gets the rotation position of the azimuth module
    public double getAzimuthRotation() {
        return mAzimuthAbsoluteEncoder.getPosition();
    }

    // Gets the x/y location of the module relative to the center of the robot
    @Override
    public Translation2d getModuleLocation() {
        return mLocation;
    }

    // Get the position of swerve modules (distance and angle)
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromDegrees(getAzimuthRotation()));
    }

    // Get the state (speed/rotation) of the swerve module
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAzimuthRotation()));
    }

    // Log swerve data
    @Override
    public void log() {

    }
}
