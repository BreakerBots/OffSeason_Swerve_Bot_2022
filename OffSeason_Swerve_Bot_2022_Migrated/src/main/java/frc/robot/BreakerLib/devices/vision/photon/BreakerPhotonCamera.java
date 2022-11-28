// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photon;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.BreakerLib.devices.BreakerGenericDeviceBase;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Photon camera */
public class BreakerPhotonCamera extends BreakerGenericDeviceBase {

    private PhotonCamera camera;
    private final String cameraName;
    private double cameraMountPitch; // Mounting angle
    private double cameraHeightMeters; // Height relative to ground.
    private Transform3d cameraPositionRelativeToRobot; // Height relative to ground, all else relative to robot position.

    /**
     * Creates a new camera that uses a PhotonVision-based computer vision
     * algorithem
     * 
     * @param cameraName                    Name of camera used to retreive data.
     * @param cameraPositionRelativeToRobot Transformation between robot center and
     *                                      camera position (Z translation is
     *                                      relative to the ground).
     */
    public BreakerPhotonCamera(String cameraName, Transform3d cameraPositionRelativeToRobot) {
        camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        deviceName = cameraName;
        this.cameraPositionRelativeToRobot = cameraPositionRelativeToRobot;
        this.cameraMountPitch = cameraPositionRelativeToRobot.getRotation().getY();
        this.cameraHeightMeters = cameraPositionRelativeToRobot.getZ();
    }

    /** @return Overall raw result from photon camera. */
    public PhotonPipelineResult getLatestRawResult() {
        return camera.getLatestResult();
    }

    /** @return If camera is locked onto any targets. */
    public boolean hasTargets() {
        return getLatestRawResult().hasTargets();
    }

    /**
     * @return List of all raw PhotonTrackedTargets the camera has in its field of
     * view.
     */
    public PhotonTrackedTarget[] getAllRawTrackedTargets() {
        return getLatestRawResult().targets.toArray(new PhotonTrackedTarget[getLatestRawResult().targets.size()]);
    }

    /** @return Number of camera targets currently locked on. */
    public int getNumberOfCameraTargets() {
        return getAllRawTrackedTargets().length;
    }

    /** @return Camera latency in milliseconds */
    public double getPipelineLatancyMilliseconds() {
        return getLatestRawResult().getLatencyMillis();
    }

    /** Sets camera pipeline based on given number. */
    public void setPipelineNumber(int pipeNum) {
        camera.setPipelineIndex(pipeNum);
    }

    /** @return Index of the active pipeline on the camera as an integer. */
    public int getCurrentPipelineNumber() {
        return camera.getPipelineIndex();
    }

    /**
     * @return The raw PhotonTrackedTarget object representing the best tracked
     * target according to the pipeline's native sort
     */
    public PhotonTrackedTarget getBestTarget() {
        return getLatestRawResult().getBestTarget();
    }

    /** @return Height relative to ground in meters. */
    public double getCameraHeight() {
        return cameraHeightMeters;
    }

    /** @return pitch of camera in degrees. */
    public double getCameraPitch() {
        return cameraMountPitch;
    }

    /** @return 2d pose of camera relative to robot. */
    public Transform2d getCamPositionRelativeToRobot() {
        return new Transform2d(cameraPositionRelativeToRobot.getTranslation().toTranslation2d(), cameraPositionRelativeToRobot.getRotation().toRotation2d());
    }

    /** @return 3d pose of camera relative to robot. */
    public Transform3d get3dCamPositionRelativeToRobot() {
        return cameraPositionRelativeToRobot;
    }

    /** Change camera position relative to robot. */
    public void updateCamPositionRelativeToRobot(Transform3d newTransform) {
        cameraPositionRelativeToRobot = newTransform;
        cameraMountPitch = Math.toDegrees(newTransform.getRotation().getY());
        cameraHeightMeters = newTransform.getTranslation().getZ();
    }

    /** Turn on or tunn off LEDs. */
    public void setLEDMode(VisionLEDMode ledMode) {
        camera.setLED(ledMode);
    }

    /** @return Current state of camera LEDs. */
    public VisionLEDMode getCurrentLEDMode() {
        return camera.getLEDMode();
    }

    @Override
    public void runSelfTest() {
        faultStr = null;
        health = DeviceHealth.NOMINAL;
        if (getPipelineLatancyMilliseconds() == 0) {
            health = DeviceHealth.INOPERABLE;
            faultStr = " camera_not_connected ";
        }
    }

    @Override
    public String getDeviceName() {
        return cameraName;
    }

    @Override
    // DOES NOUTHING, exists to satisfy BreakerGenericDevice Interface
    public void setDeviceName(String newName) {}

    @Override
    public boolean isUnderAutomaticControl() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public DevicePowerMode getPowerMode() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void returnToAutomaticPowerManagement() {
        // TODO Auto-generated method stub
        
    }

}
