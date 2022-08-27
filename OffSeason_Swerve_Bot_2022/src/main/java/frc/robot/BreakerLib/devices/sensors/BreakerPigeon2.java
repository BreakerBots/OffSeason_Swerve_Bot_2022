// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.BreakerLib.devices.sensors;

import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.devices.BreakerGenericDeviceBase;
import frc.robot.BreakerLib.position.geometry.BreakerRotation3d;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.selftest.SelfTest;

/* CTRE Pigeon IMU 2 implementing the Breaker device interface. */
public class BreakerPigeon2 extends BreakerGenericDeviceBase {
  private WPI_Pigeon2 pigeon;

  /** Creates a new PigeonIMU object. */
  public BreakerPigeon2(int deviceID) {
    pigeon = new WPI_Pigeon2(deviceID);
    deviceName = "Pigeon2_IMU (" + deviceID + ") ";
  }

  /** Returns pitch angle within +- 180 degrees */
  public double getPitchDegrees() {
    return BreakerMath.angleModulus(pigeon.getPitch());
  }

  /** Returns yaw angle within +- 180 degrees */
  public double getYawDegrees() {
    return BreakerMath.angleModulus(pigeon.getYaw());
  }

  /** Returns roll angle within +- 180 degrees */
  public double getRollDegrees() {
    return BreakerMath.angleModulus(pigeon.getRoll());
  }

  /** Pitch within +- 180 degrees as Rotation2d. */
  public Rotation2d getPitchRotation2d() {
    return Rotation2d.fromDegrees(getPitchDegrees());
  }

  /** Yaw within +- 180 degrees as Rotation2d. */
  public Rotation2d getYawRotation2d() {
    return Rotation2d.fromDegrees(getYawDegrees());
  }

  /** Roll within +- 180 degrees as Rotation2d. */
  public Rotation2d getRollRotation2d() {
    return Rotation2d.fromDegrees(getRollDegrees());
  }

  /** Pitch, yaw, and roll as Rotation3d, all within +- 180 degrees. */
  public BreakerRotation3d getRotation3d() {
    return new BreakerRotation3d(getPitchRotation2d(), getYawRotation2d(), getRollRotation2d());
  }

  /**
   * Returns raw yaw, pitch, and roll angles in an array.
   * <p>
   * yaw = 0, pitch = 1, roll = 2.
   */
  public double[] getRawAngles() {
    double[] RawYPR = new double[3];
    pigeon.getYawPitchRoll(RawYPR);
    return RawYPR;
  }

  /** Resets yaw to 0 degrees */
  public void reset() {
    pigeon.setYaw(0);
  }

  /** Sets yaw to given angle. */
  public void set(double angle) {
    pigeon.setYaw(angle);
  }

  /** Returns accelerometer value based on given index */
  public double getGyroRates(int arrayElement) {
    double[] rawRates = new double[3];
    pigeon.getRawGyro(rawRates);
    return rawRates[arrayElement];
  }

  /** Accelerometer pitch. */
  public double getPitchRate() {
    return getGyroRates(0);
  }

  /** Accelerometer yaw. */
  public double getYawRate() {
    return getGyroRates(1);
  }

  /** Accelerometer roll. */
  public double getRollRate() {
    return getGyroRates(2);
  }

  /**
   * Returns array of raw accelerometer values.
   * <p>
   * x = 0, y = 1, z = 2.
   */
  public short[] getRawAccelerometerVals() {
    short[] accelVals = new short[3];
    pigeon.getBiasedAccelerometer(accelVals);
    return accelVals;
  }

  /** Accelerometer x-value in inches */
  public double getRawIns2AccelX() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals()[0], 14) * 0.02);
  }

  /** Accelerometer y-value in inches */
  public double getRawIns2AccelY() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals()[1], 14) * 0.02);
  }

  /** Accelerometer z-value in inches */
  public double getRawIns2AccelZ() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals()[2], 14) * 0.02);
  }

  /** How long the Pigeon has been running for, in seconds. Maxes out at 255 sec.*/
  public int getPigeonUpTime() {
    return pigeon.getUpTime();
  }

  public BreakerRotation3d getRawRotation3d() {
    return new BreakerRotation3d(Rotation2d.fromDegrees(getRawAngles()[1]), Rotation2d.fromDegrees(getRawAngles()[0]),
        Rotation2d.fromDegrees(getRawAngles()[2]));
  }

  @Override
  public void runSelfTest() {
    faultStr = null;
    health = DeviceHealth.NOMINAL;
    Pigeon2_Faults curFaults = new Pigeon2_Faults();
    pigeon.getFaults(curFaults);
    
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
