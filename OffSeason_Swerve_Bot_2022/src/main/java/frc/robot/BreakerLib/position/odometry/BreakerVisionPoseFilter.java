// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.KalmanFilterLatencyCompensator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.BreakerLib.devices.vision.photonvision.BreakerFiducialPhotonTarget;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerVisionPoseFilter {
    private BreakerFiducialPhotonTarget[] positioningTargets;
    private double trustCoef, maxUncertanty;
    /**Fiters all predcted poses from visable Fiducial targets through a weaighted average. Weaights calculated by: 
     * trustCoef ^ ((-trustCoef) * poseUncertanty)
     * 
     * @param trustCoef - higher values mean more uncertan values are truested less
     * @param maxUncertanty - the highest uncirtanty value (0-1) that will still be considered in the pose calculation
     * @param positioningTargets - the fiducial targets for positioning
     */
    public BreakerVisionPoseFilter(double trustCoef, double maxUncertanty, BreakerFiducialPhotonTarget... positioningTargets) {
        this.positioningTargets = positioningTargets;
        this.trustCoef = MathUtil.clamp(trustCoef, 1, Double.MAX_VALUE);
        this.maxUncertanty = maxUncertanty;
    }

    public Pose3d getFilteredRobotPose3d() {
        List<Double> weights = new ArrayList<>();
        List<Pose3d> predictedPoses = new ArrayList<>();
        for (int i = 0; i < positioningTargets.length; i++) {
            BreakerFiducialPhotonTarget tgt = positioningTargets[i];
            if (tgt.isAssignedTargetVisable()) {
                if (tgt.getPoseAmbiguity() <= maxUncertanty) {
                    double weight = MathUtil.clamp(Math.pow(trustCoef,(-trustCoef)*tgt.getPoseAmbiguity()), 0.0, 1.0);
                    weights.add(weight);
                    predictedPoses.add(tgt.getRobotPose3d());
                }
            }
        }

        double[] weightArr = new double[weights.size()];
        double[] xArr = new double[predictedPoses.size()];
        double[] yArr = new double[predictedPoses.size()];
        double[] zArr = new double[predictedPoses.size()];
        double[] yAngArr = new double[predictedPoses.size()]; //in rad
        double[] pAngArr = new double[predictedPoses.size()]; //in rad
        double[] rAngArr = new double[predictedPoses.size()]; //in rad

        for (int i = 0; i < weights.size(); i++) {
            weightArr[i] = weights.get(i);

            xArr[i] = predictedPoses.get(i).getX();
            yArr[i] = predictedPoses.get(i).getY();
            zArr[i] = predictedPoses.get(i).getZ();

            yAngArr[i] = predictedPoses.get(i).getRotation().getZ();
            pAngArr[i] = predictedPoses.get(i).getRotation().getY();
            rAngArr[i] = predictedPoses.get(i).getRotation().getX();
        }

        double x = BreakerMath.getWeightedAvg(xArr, weightArr);
        double y = BreakerMath.getWeightedAvg(yArr, weightArr);
        double z = BreakerMath.getWeightedAvg(zArr, weightArr);

        double yaw = BreakerMath.getWeightedAvg(yAngArr, weightArr);
        double pitch = BreakerMath.getWeightedAvg(pAngArr, weightArr);
        double roll = BreakerMath.getWeightedAvg(rAngArr, weightArr);

        return new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
    }

    public Pose2d getFilteredRobotPose() {
        // note, does not simply convert from 3d pose to conserve CPU time
        List<Double> weights = new ArrayList<>();
        List<Pose2d> predictedPoses = new ArrayList<>();
        for (int i = 0; i < positioningTargets.length; i++) {
            BreakerFiducialPhotonTarget tgt = positioningTargets[i];
            if (tgt.isAssignedTargetVisable()) {
                if (tgt.getPoseAmbiguity() <= maxUncertanty) {
                    double weight = MathUtil.clamp(Math.pow(trustCoef,(-trustCoef)*tgt.getPoseAmbiguity()), 0.0, 1.0);
                    weights.add(weight);
                    predictedPoses.add(tgt.getRobotPose());
                }
            }
        }

        double[] weightArr = new double[weights.size()];
        double[] xArr = new double[predictedPoses.size()];
        double[] yArr = new double[predictedPoses.size()];
        double[] yAngArr = new double[predictedPoses.size()]; //in rad
        
        for (int i = 0; i < weights.size(); i++) {
            weightArr[i] = weights.get(i);

            xArr[i] = predictedPoses.get(i).getX();
            yArr[i] = predictedPoses.get(i).getY();
            yAngArr[i] = predictedPoses.get(i).getRotation().getRadians();
        }

        double x = BreakerMath.getWeightedAvg(xArr, weightArr);
        double y = BreakerMath.getWeightedAvg(yArr, weightArr);
        double yaw = BreakerMath.getWeightedAvg(yAngArr, weightArr);

        return new Pose2d(x, y, new Rotation2d(yaw));
    }

    
}
