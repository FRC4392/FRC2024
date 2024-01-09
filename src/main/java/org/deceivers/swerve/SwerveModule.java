package org.deceivers.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    /**
     * Run any initialization needed on the swerve module
     */
    void init();
    /**
     * Log necassary data to driverstation and file
     */
    void log();
    /**
     * Set the state the swerve drive should attempt to be in
     */
    void set(SwerveModuleState drive);
    void setClosedLoop(SwerveModuleState drive);
    /**
     * Stop all motion on the swerve drive.
     */
    void stop();
    /**
     * Return the current state of the swerve module
     */
    SwerveModuleState getState();

    SwerveModulePosition getPosition();
    /**
     * Get the location of the swerve module relative to the 
     * center of the robot
     */
    Translation2d getModuleLocation();
	void setAngle(double angle);
}
