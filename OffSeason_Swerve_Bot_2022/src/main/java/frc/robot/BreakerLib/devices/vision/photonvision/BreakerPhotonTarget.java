// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photonvision;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;

/** Add your docs here. */
public class BreakerPhotonTarget {
    private BreakerPhotonCamera camera;
    private Pose3d targetPose;
    private boolean isFiducical = true, isSupplied = false;
    private int feducialID;

    private double maxCordnateDev;
    private BreakerGenericOdometer odometer;

    private Supplier<PhotonTrackedTarget> assignedTargetSupplier;
    public BreakerPhotonTarget(BreakerPhotonCamera camera, Pose3d targetPose, int fiducicalID) {
        
    }

    public BreakerPhotonTarget(BreakerPhotonCamera camera, Pose3d targetPose, double maxCordnateDev, BreakerGenericOdometer odometer) {
        
    }

    public BreakerPhotonTarget(BreakerPhotonCamera camera, Pose3d targetPose, Supplier<PhotonTrackedTarget> assignedTargetSupplier) {

    }
}
