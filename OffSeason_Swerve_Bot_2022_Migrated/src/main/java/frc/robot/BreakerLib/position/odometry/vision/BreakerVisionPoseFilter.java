// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.devices.vision.photonvision.BreakerFiducialPhotonTarget;
import frc.robot.BreakerLib.util.math.averages.BreakerAverage;
import frc.robot.BreakerLib.util.math.averages.BreakerWeightedAverage;

/** Weighted average for vision-based pose predictions. */
public class BreakerVisionPoseFilter {
    private BreakerFiducialPhotonTarget[] positioningTargets;
    private BreakerAverage avgLatency;
    private double trustCoef, maxUncertanty;

    /**
     * Fiters all predcted poses from visable Fiducial targets through a weaighted
     * average. Weaights calculated by:
     * trustCoef ^ ((-trustCoef) * poseUncertanty)
     * 
     * @param trustCoef          - Higher values mean more uncertain values are
     *                           trusted less.
     * @param maxUncertanty      - The highest uncertainty value (0-1) that will
     *                           still be considered in the pose calculation.
     * @param positioningTargets - The fiducial targets for positioning.
     */
    public BreakerVisionPoseFilter(double trustCoef, double maxUncertanty,
            BreakerFiducialPhotonTarget... positioningTargets) {
        this.positioningTargets = positioningTargets;
        this.trustCoef = MathUtil.clamp(trustCoef, 1, Double.MAX_VALUE);
        this.maxUncertanty = maxUncertanty;
        avgLatency = new BreakerAverage();
    }

    /** @return Robot pose with weighted average applied. */
    public Pose3d getFilteredRobotPose3d() {
        BreakerWeightedAverage xAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage zAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAngAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage pAngAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage rAngAverage = new BreakerWeightedAverage();
        avgLatency.clear();
        for (int i = 0; i < positioningTargets.length; i++) {
            BreakerFiducialPhotonTarget tgt = positioningTargets[i];
            if (tgt.isAssignedTargetVisible()) {
                if (tgt.getPoseAmbiguity() <= maxUncertanty) {
                    double weight = MathUtil.clamp(Math.pow(trustCoef, (-trustCoef) * tgt.getPoseAmbiguity()), 0.0,
                            1.0);
                    Pose3d pose = tgt.getRobotPose3d();
                    xAverage.addValue(pose.getX(), weight);
                    yAverage.addValue(pose.getY(), weight);
                    zAverage.addValue(pose.getZ(), weight);
                    yAngAverage.addValue(pose.getRotation().getZ(), weight);
                    pAngAverage.addValue(pose.getRotation().getY(), weight);
                    rAngAverage.addValue(pose.getRotation().getX(), weight);
                    avgLatency.addValue(tgt.getTargetDataTimestamp());
                }
            }
        }

        double x = xAverage.getAverage();
        double y = yAverage.getAverage();
        double z = zAverage.getAverage();

        double yaw = yAngAverage.getAverage();
        double pitch = pAngAverage.getAverage();
        double roll = rAngAverage.getAverage();

        return new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
    }

    public Pose2d getFilteredRobotPose() {
        // note, does not simply convert from 3d pose to conserve CPU time
        BreakerWeightedAverage xAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAverage = new BreakerWeightedAverage();
        BreakerWeightedAverage yAngAverage = new BreakerWeightedAverage();
        avgLatency.clear();
        for (int i = 0; i < positioningTargets.length; i++) {
            BreakerFiducialPhotonTarget tgt = positioningTargets[i];
            if (tgt.isAssignedTargetVisible()) {
                if (tgt.getPoseAmbiguity() <= maxUncertanty) {
                    double weight = MathUtil.clamp(Math.pow(trustCoef, (-trustCoef) * tgt.getPoseAmbiguity()), 0.0,
                            1.0);
                    Pose2d pose = tgt.getRobotPose();
                    xAverage.addValue(pose.getX(), weight);
                    yAverage.addValue(pose.getY(), weight);
                    yAngAverage.addValue(pose.getRotation().getRadians(), weight);
                    avgLatency.addValue(tgt.getTargetDataTimestamp());
                }
            }
        }

        double x = xAverage.getAverage();
        double y = yAverage.getAverage();
        double yaw = yAngAverage.getAverage();
      
        return new Pose2d(x, y, new Rotation2d(yaw));
    }

    public boolean isAnyTargetVisable() {
        for (BreakerFiducialPhotonTarget tgt: positioningTargets) {
            if (tgt.isAssignedTargetVisible()) {
                return true;
            }
        }
        return false;
    }

    public double getDataTimestamp() {
        return avgLatency.getAverage();
    }

}
