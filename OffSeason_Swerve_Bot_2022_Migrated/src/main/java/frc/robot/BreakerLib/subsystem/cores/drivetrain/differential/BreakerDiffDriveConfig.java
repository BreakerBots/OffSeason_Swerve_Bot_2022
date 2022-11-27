// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;

/** Configuration for {@link BreakerDiffDrive}. */
public class BreakerDiffDriveConfig {
    private DifferentialDriveKinematics kinematics;
    private double robotTrackWidthInches;
    private double ticksPerEncoderRotation;
    private double ticksPerInch;
    private double gearRatioTo1;
    private double wheelDiameter;
    private double wheelCircumference;
    private double getTicksPerWheelRotation;
    private double slowModeForwardMultiplier = 1;
    private double slowModeTurnMultiplier = 1;

    public BreakerDiffDriveConfig() {
        
    }

    /**
     * Creates a config object for BreakerDiffDrive.
     */
    public BreakerDiffDriveConfig(double ticksPerEncoderRotation, double gearRatioTo1, double wheelDiameter, double robotTrackWidthInches) {

        this.wheelDiameter = wheelDiameter;
        this.ticksPerEncoderRotation = ticksPerEncoderRotation;
        this.gearRatioTo1 = gearRatioTo1;
        this.robotTrackWidthInches = robotTrackWidthInches;

        kinematics = new DifferentialDriveKinematics(BreakerUnits.inchesToMeters(robotTrackWidthInches));

        wheelCircumference = BreakerMath.getCircumferenceFromDiameter(wheelDiameter);
        ticksPerInch = BreakerMath.getTicksPerInch(ticksPerEncoderRotation, gearRatioTo1, wheelDiameter);
        getTicksPerWheelRotation = BreakerMath.getTicksPerRotation(ticksPerEncoderRotation, gearRatioTo1);
    }

    public void setSlowModeMultipliers(double forwardMult, double turnMult) {
        slowModeForwardMultiplier = forwardMult;
        slowModeTurnMultiplier = turnMult;
    }

    public double getTicksPerInch() {
        return ticksPerInch;
    }

    public double getEncoderTicks() {
        return ticksPerEncoderRotation;
    }

    public double getGearRatioTo1() {
        return gearRatioTo1;
    }

    public double getGetTicksPerWheelRotation() {
        return getTicksPerWheelRotation;
    }

    public double getWheelCircumference() {
        return wheelCircumference;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public double getRobotTrackWidthInches() {
        return robotTrackWidthInches;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getSlowModeForwardMultiplier() {
        return slowModeForwardMultiplier;
    }

    public double getSlowModeTurnMultiplier() {
        return slowModeTurnMultiplier;
    }
}
