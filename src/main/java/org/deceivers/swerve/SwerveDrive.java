package org.deceivers.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDrive extends SubsystemBase{
    //Swerve Devices
    private final SwerveModule[] mModules;
    private final int numModules;
    private final SwerveDriveKinematics mKinematics;
    private final SwerveDrivePoseEstimator mSwerveDrivePoseEstimator;
    private final DoubleSupplier mGyroAngle;
    private final ProfiledPIDController rotationPIDController = new ProfiledPIDController(15,.1,.1,new TrapezoidProfile.Constraints(500, 500));
    private double gyroOffset = 0;
    private double speedScalar = .5;


    public SwerveDrive(DoubleSupplier gyroAngle, SwerveModule... modules){

        mGyroAngle = gyroAngle;
        numModules = modules.length;
        mModules = Arrays.copyOf(modules, numModules);

        
        //Configure Kinematics
        Translation2d[] moduleLocations = new Translation2d[numModules];
        for (int i = 0; i < numModules; i++){
            moduleLocations[i] = mModules[i].getModuleLocation();
        }
        mKinematics = new SwerveDriveKinematics(moduleLocations);

        //Configure PID Controllers
        rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

        //mDriveController = new HolonomicDriveController(new PIDController(4,0,0), new PIDController(4,0,0), rotationPIDController);

        //Configure Pose Estimator
        SwerveModulePosition[] states = new SwerveModulePosition[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getPosition();
        }
        mSwerveDrivePoseEstimator = new SwerveDrivePoseEstimator(mKinematics, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()), states, new Pose2d(), VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(360)));

        //Initialize Swerve Modules
        Arrays.stream(mModules).forEach(SwerveModule::init);
    }

    //Set new pose location
    public void setLocation(double x, double y, double angle){
        Pose2d newPose = new Pose2d(x, y, Rotation2d.fromDegrees(angle));
        SwerveModulePosition[] states = new SwerveModulePosition[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getPosition();
        }

        mSwerveDrivePoseEstimator.resetPosition(Rotation2d.fromDegrees(mGyroAngle.getAsDouble()), states, newPose);
        rotationPIDController.reset(angle);
    }

    //Set a module to a specific angle
    public void setModulesAngle(double angle, int module){
        mModules[module].setAngle(angle);
    }

    //Get the current estimated pose
    public Pose2d getPose(){
        return mSwerveDrivePoseEstimator.getEstimatedPosition();
    }

    //Add vision measurement to pose
    public void updatePoseWithVision(Pose2d newPose, double time){
        mSwerveDrivePoseEstimator.addVisionMeasurement(newPose, time);
    }

    //get the current speed of the chassis
    public ChassisSpeeds getChassisSpeeds(){
        SwerveModuleState[] states = new SwerveModuleState[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getState();
        }
        ChassisSpeeds chassisSpeeds = mKinematics.toChassisSpeeds(states);
        return(chassisSpeeds);
    }

    //Stop movement of all swerve modules
	public void stop() {
        Arrays.stream(mModules).forEach(SwerveModule::stop);
	}

    //Control swerve modules
    public void drive(double forward, double strafe, double azimuth, boolean fieldRelative){
        ChassisSpeeds speeds;
        if (fieldRelative){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, azimuth, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));
        } else {
            speeds = new ChassisSpeeds(forward, strafe, azimuth);
        }
        SwerveModuleState[] states = mKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1);
        for (int i = 0; i < numModules; i++){
            mModules[i].set(states[i]);
        }
    }

    //control swerve modules in closed loop control
    public void driveClosedLoop(double forward, double strafe, double azimuth, boolean fieldRelative){
        ChassisSpeeds speeds;
        if (fieldRelative){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, azimuth, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));
        } else {
            speeds = new ChassisSpeeds(forward, strafe, azimuth);
        }
        SwerveModuleState[] states = mKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 5);
        for (int i = 0; i < numModules; i++){
            mModules[i].setClosedLoop(states[i]);
        }
    }

    public void resetGyro(){
        setGyro(0);
      }
  
      public void setGyro(double position){
        gyroOffset = (position - mGyroAngle.getAsDouble());
      }

    public double getRotation() {
        return mGyroAngle.getAsDouble() + gyroOffset;
      }

    public double getYaw(){
        return mGyroAngle.getAsDouble();
    }

    //Update odometry readings
    public Pose2d updateOdometry(){
        SwerveModulePosition[] states = new SwerveModulePosition[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getPosition();
        }

        return mSwerveDrivePoseEstimator.update(Rotation2d.fromDegrees(mGyroAngle.getAsDouble()), states);
    }

    public void log(){
        Arrays.stream(mModules).forEach(SwerveModule::log);
        //double[] swervePoseArray = {getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()};
    }

    public Command brakeCommand(){
      return this.run(() -> {
        double angle = -45;
        for (int i = 0; i < 4; i++){
          setModulesAngle(angle, i);
          angle += 90;
        }
      });
    }

    public Command restGyro(){
        return this.runOnce(this::restGyro);
    }

    public Command FastFieldRelativeDrive(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier rotVelocity){
        return this.run(() -> {
            this.drive(yVelocity.getAsDouble(), xVelocity.getAsDouble(), rotVelocity.getAsDouble(), true); 
        });
    }

    public Command FastRobotdRelativeDrive(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier rotVelocity){
        return this.run(() -> {
            this.drive(yVelocity.getAsDouble(), xVelocity.getAsDouble(), rotVelocity.getAsDouble(), false); 
        });
    }

    public Command SlowFieldRelativeDrive(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier rotVelocity){
        return this.run(() -> {
            this.drive(yVelocity.getAsDouble()*speedScalar, xVelocity.getAsDouble()*speedScalar, rotVelocity.getAsDouble()*speedScalar, true); 
        });
    }

    public Command SlowRobotdRelativeDrive(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier rotVelocity){
        return this.run(() -> {
            this.drive(yVelocity.getAsDouble()*speedScalar, xVelocity.getAsDouble()*speedScalar, rotVelocity.getAsDouble()*speedScalar, false); 
        });
    }

}