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
    public BreakerPhotonTarget(BreakerPhotonCamera camera, Pose3d targetPose, int feducicalID) {
        
    }

    public BreakerPhotonTarget(BreakerPhotonCamera camera, Pose3d targetPose, BreakerGenericOdometer odometer) {
        
    }

    public BreakerPhotonTarget(BreakerPhotonCamera camera, Pose3d targetPose, Supplier<PhotonTrackedTarget> assignedTargetSupplier) {

    }
}
