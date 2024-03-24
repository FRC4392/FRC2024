package org.deceivers.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleV3 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final AbsoluteEncoder mAzimuthAbsoluteEncoder;
    private final RelativeEncoder mAzimuthIncrementalEncoder;
    private final SparkPIDController mAzimuthPID;

    private final TalonFX mDriveMotor;
    private final VelocityVoltage mDriveVelocityControl;
    private final StatusSignal<Double> mDrivePositionSignal;
    private final StatusSignal<Double> mDriveVelocitySignal;
    private final VoltageOut mDriveVoltageControl;

    private final Translation2d mLocation;
    
    @SuppressWarnings("unused")
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
        mDriveVoltageControl = new VoltageOut(0);
        mDrivePositionSignal = mDriveMotor.getPosition();
        mDriveVelocitySignal = mDriveMotor.getVelocity();

        // Get Azimuth Controller and Encoders
        mAzimuthPID = mAzimuthMotor.getPIDController();
        mAzimuthAbsoluteEncoder = mAzimuthMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        mAzimuthIncrementalEncoder = mAzimuthMotor.getEncoder();

        //Configure Drive motor
        mDriveMotor.getConfigurator().apply(SwerveConfigs.driveConfigs);

        // Configure azimuth motor controller parameters
        // Rest motors to factory defaults to ensure correct parameters
        mAzimuthMotor.restoreFactoryDefaults();

        // Configure azimuth motor controller parameters
        mAzimuthMotor.setInverted(SwerveConfigs.azimuthIsInverted);
        mAzimuthMotor.setIdleMode(SwerveConfigs.azimuthIdleMode);
        mAzimuthMotor.setSmartCurrentLimit(SwerveConfigs.azimuthCurrentLimit);

        // Configure drive absolute encoder
        mAzimuthAbsoluteEncoder.setPositionConversionFactor(SwerveConfigs.azimuthAbsolutePositionConversionFactor);
        mAzimuthAbsoluteEncoder.setInverted(SwerveConfigs.azimuthAbsoluteInverted);
        mAzimuthAbsoluteEncoder.setAverageDepth(SwerveConfigs.azimuthAbsoluteSamplingDepth);

        // Configure azimuth incremental encoder
        mAzimuthIncrementalEncoder.setPositionConversionFactor(SwerveConfigs.azimuthIncrementalPositionConversionFactor);

        // Configure azimuth PID
        mAzimuthPID.setFeedbackDevice(mAzimuthAbsoluteEncoder);
        mAzimuthPID.setP(SwerveConfigs.azimuthPIDkP);
        mAzimuthPID.setPositionPIDWrappingEnabled(SwerveConfigs.azimuthPIDwrapping);
        mAzimuthPID.setPositionPIDWrappingMinInput(SwerveConfigs.azimuthPIDMinWrap);
        mAzimuthPID.setPositionPIDWrappingMaxInput(SwerveConfigs.azimuthPIDMaxWrap);

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

        SmartDashboard.putNumber(mName + " Velocity", mDriveMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(mName + " Position", mDriveMotor.getPosition().getValueAsDouble());

    }

    public void setVoltage(double voltage){
        mDriveMotor.setControl(mDriveVoltageControl.withOutput(voltage));
    }

    public void driveVoltage(double voltage){
        setAngle(0);
        mDriveMotor.setControl(mDriveVelocityControl.withVelocity(voltage));
    }
}
