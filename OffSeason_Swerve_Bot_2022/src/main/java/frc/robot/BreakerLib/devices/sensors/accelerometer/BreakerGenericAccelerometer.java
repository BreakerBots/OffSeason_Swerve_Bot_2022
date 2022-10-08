// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.accelerometer;

/** FRC 3-axis accelerometer interface. */
public interface BreakerGenericAccelerometer {
  /**
   * Returns array of raw accelerometer values.
   * <p>
   * x = 0, y = 1, z = 2.
   */
  public short[] getRawAccelerometerVals();

  /** @return Unbiased accelerometer x-value in m/s^2 */
  public double getRawAccelX();

  /** @return Unbiased accelerometer y-value in m/s^2 */
  public double getRawAccelY();

  /** @return Unbiased accelerometer z-value in m/s^2 */
  public double getRawAccelZ();
}
