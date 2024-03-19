package org.deceivers.swerve;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkRelativeEncoder.Type;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleV3 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final TalonFX mDriveMotor;
    private final AbsoluteEncoder mAzimuthAbsoluteEncoder;
    private final RelativeEncoder mAzimuthIncrementalEncoder;
    //private final RelativeEncoder mDriveEncoder; (for NEO)
    //private final SparkPIDController mDrivePID; (for NEO)
    private final SparkPIDController mAzimuthPID;
    private final Translation2d mLocation;
    private final String mName;

    // need to update the speed to m/s

    public SwerveModuleV3(CANSparkMax azimuthMotor, TalonFX driveMotor,
            Translation2d location, String name) {

        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
        mLocation = location;
        mName = name;

        // Rest motors to factory defaults to ensure correct parameters
        mAzimuthMotor.restoreFactoryDefaults();

        // Get encoders
        mAzimuthAbsoluteEncoder = mAzimuthMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        mAzimuthIncrementalEncoder = mAzimuthMotor.getEncoder();
        //mDriveEncoder = mDriveMotor.getEncoder(Type.kQuadrature, 7168);
        //mDriveEncoder.setAverageDepth(1);
        //mDriveEncoder.setMeasurementPeriod(1);

        // Get Azimuth PIDs
        mAzimuthPID = mAzimuthMotor.getPIDController();

        // Configure drive motor controller parameters
        CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs();

            driveCurrentLimits.SupplyCurrentLimit = 80;
            driveCurrentLimits.SupplyCurrentThreshold = 80;
            driveCurrentLimits.SupplyTimeThreshold = 1;
            driveCurrentLimits.StatorCurrentLimit = 80;

            driveCurrentLimits.StatorCurrentLimitEnable = true;
            driveCurrentLimits.SupplyCurrentLimitEnable = true;

        MotorOutputConfigs driveOutputConfigs = new MotorOutputConfigs();

            driveOutputConfigs.withInverted(InvertedValue.Clockwise_Positive);

                
        mDriveMotor.setNeutralMode(NeutralModeValue.Brake);

//Configure Drive PID
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();

        driveConfigs.Feedback.SensorToMechanismRatio = ((0.2393893602 / ((20.0 / 13.0) * (45.0 / 15.0))) / 60);

        driveConfigs.Slot0.kV = 0; // velocity speed from smart dashboard/11
        driveConfigs.Slot0.kP = 0; // proportional
        driveConfigs.Slot0.kI = 0; // integral
        driveConfigs.Slot0.kD = 0; // derivative

        driveConfigs.CurrentLimits = driveCurrentLimits;
        driveConfigs.MotorOutput = driveOutputConfigs;

        mDriveMotor.getConfigurator().apply(driveConfigs);


        // Configure drive motor controller parameters (for NEO)    
        // mDriveMotor.setInverted(true);
        //mDriveMotor.setClosedLoopRampRate(0);
        // mDriveMotor.setOpenLoopRampRate(.1);
        // mDriveMotor.setIdleMode(IdleMode.kBrake);
        // mDriveMotor.setSmartCurrentLimit(80);

        // Configure Drive Encoder (for NEO)
        // mDriveEncoder.setPositionConversionFactor(0.2393893602 / ((20.0 * 45.0) / (13.0 * 15.0)));
        // mDriveEncoder.setVelocityConversionFactor(((0.2393893602 / ((20.0 / 13.0) * (45.0 / 15.0))) / 60));
        // mDriveEncoder.setPosition(0);

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

        // Configure drive PID (for NEO)
        // mDrivePID.setFF(.215);
        // mDrivePID.setP(.1);// .1

        // Configure azimuth PID
        mAzimuthPID.setFeedbackDevice(mAzimuthAbsoluteEncoder);
        mAzimuthPID.setP(.05);
        mAzimuthPID.setPositionPIDWrappingEnabled(true);
        mAzimuthPID.setPositionPIDWrappingMinInput(0);
        mAzimuthPID.setPositionPIDWrappingMaxInput(360);

        // Burn flahs in case of power cycle
        mAzimuthMotor.burnFlash();
    }

    // Sets the drive motor speed in open loop mode
    public void setSpeed(double speed) {
        mDriveMotor.set(speed);
    }

    // Sets the rotation speed of the azimuth motor in open loop mode
    public void setRotation(double rotation) {
        mAzimuthMotor.set(rotation);
    }

    // Gets the speed of the drive motor
    public double getSpeed() {
        return mDriveMotor.getVelocity().getValueAsDouble();
    }

    // Gets the rotation position of the azimuth module
    public double getRotation() {
        return mAzimuthAbsoluteEncoder.getPosition();
    }

    // Gets the x/y location of the module relative to the center of the robot
    @Override
    public Translation2d getModuleLocation() {
        return mLocation;
    }

    // Get the state (speed/rotation) of the swerve module
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getRotation()));
    }

    // Run when swerve drive is first initialized
    @Override
    public void init() {

    }

    // Get the position of swerve modules (distance and angle)
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), Rotation2d.fromDegrees(getRotation()));
    }

    // Get the distance of the drive encoder
    public double getDistance() {
        return mDriveMotor.getPosition().getValueAsDouble();
    }

    // Log swerve data
    @Override
    public void log() {
        SmartDashboard.putNumber(mName + " Current", mDriveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(mName + " Speed", mDriveMotor.getVelocity().getValueAsDouble());
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
        //mDrivePID.setReference(velocity, ControlType.kVelocity); (for NEO)

        SmartDashboard.putNumber("Closed Loop Speed " + mName, velocity);
    }

    // Stop all motors
    @Override
    public void stop() {
        mAzimuthMotor.set(0);
        mDriveMotor.set(0);
    }

    // set angle of swerve drive
    @Override
    public void setAngle(double angle) {
        mAzimuthPID.setReference(angle, ControlType.kPosition);

    }

}
