// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.BreakerLib.devices.sensors.imu.ctre;

import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.devices.sensors.imu.BreakerGenericIMU;
import frc.robot.BreakerLib.position.geometry.BreakerRotation3d;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/* CTRE Pigeon IMU 2 implementing the Breaker device interface and Breaker IMU interface. */
public class BreakerPigeon2 extends BreakerGenericIMU {
  private WPI_Pigeon2 pigeon;

  /** Creates a new PigeonIMU object. */
  public BreakerPigeon2(int deviceID) {
    pigeon = new WPI_Pigeon2(deviceID);
    deviceName = "Pigeon2_IMU (" + deviceID + ") ";
  }

  @Override
  public double getPitchDegrees() {
    return BreakerMath.angleModulus(pigeon.getPitch());
  }

  @Override
  public double getYawDegrees() {
    return BreakerMath.angleModulus(pigeon.getYaw());
  }

  @Override
  public double getRollDegrees() {
    return BreakerMath.angleModulus(pigeon.getRoll());
  }

  @Override
  public Rotation2d getPitchRotation2d() {
    return Rotation2d.fromDegrees(getPitchDegrees());
  }

  @Override
  public Rotation2d getYawRotation2d() {
    return Rotation2d.fromDegrees(getYawDegrees());
  }

  @Override
  public Rotation2d getRollRotation2d() {
    return Rotation2d.fromDegrees(getRollDegrees());
  }

  @Override
  public BreakerRotation3d getRotation3d() {
    return new BreakerRotation3d(getPitchRotation2d(), getYawRotation2d(), getRollRotation2d());
  }

  @Override
  public double[] getRawAngles() {
    double[] RawYPR = new double[3];
    pigeon.getYawPitchRoll(RawYPR);
    return RawYPR;
  }

  @Override
  public double getRawYaw() {
    return getRawAngles()[0];
  }

  @Override
  public double getRawPitch() {
    return getRawAngles()[1];
  }

  @Override
  public double getRawRoll() {
    return getRawAngles()[2];
  }

  /** Does nothing. */
  @Override
  public void setPitch(double value) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setYaw(double value) {
    pigeon.setYaw(0);
  }

  /** Does nothing. */
  @Override
  public void setRoll(double value) {

  }

  /** Sets yaw to 0 */
  @Override
  public void reset() {
    pigeon.setYaw(0);
  }

  public double[] getRawGyroRates() {
    double[] rawRates = new double[3];
    pigeon.getRawGyro(rawRates);
    return rawRates;
  }

  @Override
  public double getRawPitchRate() {
    return getRawGyroRates()[0];
  }

  @Override
  public double getRawRollRate() {
    return getRawGyroRates()[1];
  }

  @Override
  public double getRawYawRate() {
    return getRawGyroRates()[2];
  }

  @Override
  public double getPitchRate() {
    return getRawPitchRate();
  }

  @Override
  public double getYawRate() {
    return  getRawYawRate();
  }

  @Override
  public double getRollRate() {
    return getRawRollRate();
  }

  @Override
  public short[] getRawAccelerometerVals() {
    short[] accelVals = new short[3];
    pigeon.getBiasedAccelerometer(accelVals);
    return accelVals;
  }

  @Override
  public double getRawAccelX() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals()[0], 14) * 0.000508);
  }

  @Override
  public double getRawAccelY() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals()[1], 14) * 0.000508);
  }

  @Override
  public double getRawAccelZ() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals()[2], 14) * 0.000508);
  }

  public int getPigeonUpTime() {

    return pigeon.getUpTime();
  }

  @Override
  public BreakerRotation3d getRawRotation3d() {
    return new BreakerRotation3d(Rotation2d.fromDegrees(getRawAngles()[1]), Rotation2d.fromDegrees(getRawAngles()[0]),
        Rotation2d.fromDegrees(getRawAngles()[2]));
  }

  @Override
  public void runSelfTest() {
    faultStr = null;
    health = DeviceHealth.NOMINAL;
    Pigeon2_Faults curFaults = new Pigeon2_Faults();

    if (curFaults.HardwareFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " HARDWARE_FAULT ";
    }
    if (curFaults.MagnetometerFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " MAG_FAULT ";
    }
    if (curFaults.GyroFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += "  GYRO_FAULT ";
    }
    if (curFaults.AccelFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += "  ACCEL_FAULT ";
    }
    if (curFaults.UnderVoltage) {
      health = (health != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : health;
      faultStr += " UNDER_6.5V ";
    }
  }

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
  

