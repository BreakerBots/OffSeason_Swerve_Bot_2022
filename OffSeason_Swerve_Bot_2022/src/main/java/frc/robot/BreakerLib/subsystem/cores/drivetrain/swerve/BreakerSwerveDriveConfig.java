// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

/** Config class for {@link BreakerSwerveDrive}. */
public class BreakerSwerveDriveConfig {

    private double maxForwardVel;
    private double maxSidewaysVel;
    private double maxAngleVel;
    private int moduleNum;
    private double moduleAnglekP;
    private double moduleAnglekI;
    private double moduleAngleKd;
    private double moduleVelkP;
    private double moduleVelkI;
    private double moduleVelKd;
    private double moduleVelKf;
    private double driveMotorGearRatioToOne;
    private double wheelDiameter;
    private double slowModeLinearMultiplier;
    private double slowModeTurnMultiplier;
    private BreakerArbitraryFeedforwardProvider arbitraryFeedforwardProvider;

    private SwerveDriveKinematics kinematics;

    /**
     * The overall configuration for a Breaker swerve drivetrain holding all constants,
     * must be passed in.
     * 
     * @param maxForwardVel                  Max forward strafe velocity of drivetrain in
     *                                       m/s.
     * @param maxSidewaysVel                 Max horizontal strafe velocity of drivetrain in
     *                                       m/s.
     * @param maxAngVel                      Max angular velocity of the drivetrain
     *                                       in rad/s.
     * @param moduleAnglekP                  Swerve module angular kP (for PID).
     * @param moduleAnglekI                  Swerve module angular kI (for PID).
     * @param moduleAngleKd                  Swerve module angular kD (for PID).
     * @param moduleVelkP                    Swerve module drive kP (for PIDF).
     * @param moduleVelkI                    Swerve module drive kI (for PIDF).
     * @param moduleVelKd                    Swerve module drive kD (for PIDF).
     * @param moduleVelKf                    Swerve module drive kF (for PIDF).
     * @param arbitraryFeedforwardProvider
     * @param driveMotorGearRatioToOne       Gear ratio of swerve modules.
     * @param wheelDiameter                  In inches.
     * @param wheelPositionsRelativeToCenter Translation2ds assigned to each module
     *                                       in order of modules added to BreakerSwerveDrive object.
     */
    public BreakerSwerveDriveConfig(double maxForwardVel, double maxSidewaysVel, double maxAngVel,
            double moduleAnglekP, double moduleAnglekI, double moduleAngleKd, double moduleVelkP,
            double moduleVelkI, double moduleVelKd, double moduleVelKf, double driveMotorGearRatioToOne,
            double wheelDiameter, BreakerArbitraryFeedforwardProvider arbitraryFeedforwardProvider,
            Translation2d... wheelPositionsRelativeToCenter) {

        this.maxForwardVel = maxForwardVel;
        this.maxSidewaysVel = maxSidewaysVel;
        this.maxAngleVel = maxAngVel;
        this.moduleAngleKd = moduleAngleKd;
        this.moduleAnglekI = moduleAnglekI;
        this.moduleAnglekP = moduleAnglekP;
        this.moduleVelKd = moduleVelKd;
        this.moduleVelkI = moduleVelkI;
        this.moduleVelkP = moduleVelkP;
        this.wheelDiameter = wheelDiameter;
        this.driveMotorGearRatioToOne = driveMotorGearRatioToOne;
        this.moduleVelKf = moduleVelKf;
        this.arbitraryFeedforwardProvider = arbitraryFeedforwardProvider;
        slowModeLinearMultiplier = 1;
        slowModeTurnMultiplier = 1;

        moduleNum = wheelPositionsRelativeToCenter.length;
        kinematics = new SwerveDriveKinematics(wheelPositionsRelativeToCenter);
    }

    public void setSlowModeMultipliers(double linearMulitplier, double turnMultiplier) {
        slowModeLinearMultiplier = linearMulitplier;
        slowModeTurnMultiplier = turnMultiplier;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getMaxForwardVel() {
        return maxForwardVel;
    }

    public double getMaxSidewaysVel() {
        return maxSidewaysVel;
    }

    public double getMaxAngleVel() {
        return maxAngleVel;
    }

    public int getNumberOfModules() {
        return moduleNum;
    }

    public double getModuleVelkP() {
        return moduleVelkP;
    }

    public double getModuleVelkI() {
        return moduleVelkI;
    }

    public double getModuleVelKd() {
        return moduleVelKd;
    }

    public double getModuleVelKf() {
        return moduleVelKf;
    }

    public double getModuleAnglekP() {
        return moduleAnglekP;
    }

    public double getModuleAnglekI() {
        return moduleAnglekI;
    }

    public double getModuleAngleKd() {
        return moduleAngleKd;
    }

    public double getDriveMotorGearRatioToOne() {
        return driveMotorGearRatioToOne;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public int getNumberOfSwerveModules() {
        return moduleNum;
    }

    public double getSlowModeLinearMultiplier() {
        return slowModeLinearMultiplier;
    }

    public double getSlowModeTurnMultiplier() {
        return slowModeTurnMultiplier;
    }

    public BreakerArbitraryFeedforwardProvider getArbitraryFeedforwardProvider() {
        return arbitraryFeedforwardProvider;
    }
}
