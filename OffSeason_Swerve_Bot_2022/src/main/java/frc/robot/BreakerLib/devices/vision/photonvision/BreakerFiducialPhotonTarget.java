// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photonvision;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerFiducialPhotonTarget extends SubsystemBase {
    private PhotonTrackedTarget assignedTarget;
    private double lastDataUpdate = Timer.getFPGATimestamp();

    private BreakerPhotonCamera camera;
    private Pose3d targetPose;
    private boolean assignedTargetFound = false;
    private int fiducicalID;

    public BreakerFiducialPhotonTarget(BreakerPhotonCamera camera, Pose3d targetPose, int fiducicalID) {
        this.camera = camera;
        this.targetPose = targetPose;
        this.fiducicalID = fiducicalID;
    }

    private void findAssignedFiducial() {
        for (PhotonTrackedTarget prospTgt: camera.getAllRawTrackedTargets()) {
            if (prospTgt.getFiducialId() == fiducicalID) {
                assignedTarget = prospTgt;
                assignedTargetFound = true;
                lastDataUpdate = Timer.getFPGATimestamp();
            }
        }
    }

    public double getLastTargetFooundTimestamp() {
        return lastDataUpdate;
    }

    public double getTargetDataAge() {
        double timediffsec = Timer.getFPGATimestamp() - lastDataUpdate;
        return Units.millisecondsToSeconds(camera.getPipelineLatancyMilliseconds()) + timediffsec;
    }

    public Pose3d getCameraPose3d() {
        return targetPose.transformBy(assignedTarget.getCameraToTarget().inverse());
    }

    public Pose2d getCameraPose() {
        return targetPose.transformBy(assignedTarget.getCameraToTarget().inverse()).toPose2d();
    }

    public Pose3d getRobotPose3d() {
        return getCameraPose3d().transformBy(camera.get3dCamPositionRelativeToRobot().inverse());
    }

    public Pose2d getRobotPose() {
        return getRobotPose3d().toPose2d();
    }

        /** Assigned target yaw */
    public double getYaw() {
        return assignedTarget.getYaw();
    }

    /** Assigned target pitch */
    public double getPitch() {
        return assignedTarget.getPitch();
    }

    /** Assigned target skew */
    public double getSkew() {
        return assignedTarget.getSkew();
    }

    /** Assigned target area */
    public double getArea() {
        return assignedTarget.getArea();
    }

    /** List of target corner coordinates. */
    public List<TargetCorner> getTargetCorners() {
        return assignedTarget.getCorners();
    }

    /** If assigned target has been found at any point during operation */
    public boolean getAssignedTargetFound() {
        return assignedTargetFound;
    }

    public double getDistance() {
        return assignedTarget.getCameraToTarget().getTranslation().toTranslation2d().getNorm();
    }

    // public Transform3d get3dTransfromFromCamera() {
    //     return assignedTarget.getCameraToTarget();
    // }

    // public Transform2d getTransformFromCamera() {
    //     return new Transform2d(assignedTarget.getCameraToTarget().getTranslation().toTranslation2d(), assignedTarget.getCameraToTarget().getRotation().toRotation2d());
    // }

    @Override
    public void periodic() {
        if (camera.hasTargets()) {
            findAssignedFiducial();
        }
    }
}
