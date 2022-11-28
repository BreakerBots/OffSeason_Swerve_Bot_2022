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
    private double robotTrackWidthMeters;
    private double gearRatioTo1;
    private double wheelDiameterMeters;
    private double wheelCircumferenceMeters;
    private double encoderRotationsPerMeter;
    private double slowModeForwardMultiplier = 1;
    private double slowModeTurnMultiplier = 1;

    /**
     * Creates a config object for BreakerDiffDrive.
     */
    public BreakerDiffDriveConfig(double gearRatioTo1, double wheelDiameterMeters, double robotTrackWidthMeters) {

        this.wheelDiameterMeters = wheelDiameterMeters;
        this.gearRatioTo1 = gearRatioTo1;
        this.robotTrackWidthMeters = robotTrackWidthMeters;

        kinematics = new DifferentialDriveKinematics(robotTrackWidthMeters);

        wheelCircumferenceMeters = BreakerMath.getCircumferenceFromDiameter(wheelDiameterMeters);
        encoderRotationsPerMeter = (1/wheelCircumferenceMeters) * gearRatioTo1;
    }

    public void setSlowModeMultipliers(double forwardMult, double turnMult) {
        slowModeForwardMultiplier = forwardMult;
        slowModeTurnMultiplier = turnMult;
    }

    public double getEncoderRotationsPerMeter() {
        return encoderRotationsPerMeter;
    }

    public double getGearRatioTo1() {
        return gearRatioTo1;
    }

    public double getWheelCircumferenceMeters() {
        return wheelCircumferenceMeters;
    }

    public double getWheelDiameterMeters() {
        return wheelDiameterMeters;
    }

    public double getRobotTrackWidthMeters() {
        return robotTrackWidthMeters;
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
