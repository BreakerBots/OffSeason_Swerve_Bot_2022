// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.KalmanFilterLatencyCompensator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.BreakerLib.devices.vision.photonvision.BreakerFiducialPhotonTarget;

/** Add your docs here. */
public class BreakerVisionPoseFilter {
    private BreakerFiducialPhotonTarget[] positioningTargets;
    private double trustCoef, maxUncertanty;
    public BreakerVisionPoseFilter(double trustCoef, double targetDataAgeTrustCoef, double maxUncertanty, BreakerFiducialPhotonTarget... positioningTargets) {
        this.positioningTargets = positioningTargets;
        this.trustCoef = trustCoef;
        this.maxUncertanty = maxUncertanty;
    }

    // public Pose2d getFilteredRobotPose() {
    //     List<Double> weights = new ArrayList<>();
    //     List<Double> avgDataAge =new ArrayList<>();
    //     List<Pose2d> predictedPoses = new ArrayList<>();
    //     for (int i = 0; i < positioningTargets.length; i++) {
    //         BreakerFiducialPhotonTarget tgt = positioningTargets[i];
    //         double weight = Math.pow(trustCoef,-trustCoef*tgt.getPoseAmbiguity());
            
    //     }

    // }

    
}
