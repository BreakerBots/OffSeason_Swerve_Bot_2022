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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.util.BreakerRoboRIO;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;

/** Add your docs here. */
public class BreakerPhotonTarget2d extends SubsystemBase{
    private BreakerPhotonCamera camera;
    private double targetHeightMeters;
    private BreakerGenericOdometer odometryProveider;
    private PhotonTrackedTarget assignedTarget;
    private Supplier<PhotonTrackedTarget> assignedTargetSupplier;
    private boolean assignedTargetFound;
    private double targetFoundTimestamp;

    /**
     * Creates a new BreakerPhotonTarget with a given predefined target.
     * 
     * @param camera                 Photon camera.
     * @param assignedTargetSupplier Supplies photon camera target.
     * @param targetHeightInches     Target height from ground.
     */
    public BreakerPhotonTarget2d(BreakerPhotonCamera camera, Supplier<PhotonTrackedTarget> assignedTargetSupplier, double targetHeightMeters) {
        this.camera = camera;
        this.assignedTargetSupplier = assignedTargetSupplier;
        assignedTarget = assignedTargetSupplier.get();
        assignedTargetFound = (assignedTarget == null) ? false : true;
        this.targetHeightMeters = targetHeightMeters;
    }

    /** Logic used to find a target. */
    private void findAssignedTarget() {
        if (camera.hasTargets()) {
            assignedTarget = assignedTargetSupplier.get();
            assignedTargetFound = (assignedTarget == null) ? false : true;
        }
    }

    /** @return Overall distance from target to camera. */
    public double getTargetDistanceMeters() {
        return PhotonUtils.calculateDistanceToTargetMeters(BreakerUnits.inchesToMeters(camera.getCameraHeight()),
                targetHeightMeters, Math.toRadians(camera.getCameraPitch()),
                Math.toRadians(getPitch()));
    }

    /** @return the relative distances in X and Y between the target and camera */
    public Translation2d getTargetTranslationFromCamera() {
        return PhotonUtils.estimateCameraToTargetTranslation(getTargetDistanceMeters(),
                Rotation2d.fromDegrees(getYaw()));
    }

    /**
     * @return the calculated X and Y cordnates of the target relative to the field
     *         based on vision and odometry
     */
    public Translation2d getTargetTranslationFromField() {
        return odometryProveider.getOdometryPoseMeters().getTranslation().plus(getTargetTranslationFromCamera());
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

    /** If assigned target is found. */
    public boolean getAssignedTargetFound() {
        return assignedTargetFound;
    }

    public double getTargetDataAge() {
        double timediffsec = Timer.getFPGATimestamp() - targetFoundTimestamp;
        return Units.millisecondsToSeconds(camera.getPipelineLatancyMilliseconds()) + timediffsec;
    }

    @Override
    public void periodic() {
        findAssignedTarget();
    }
}
