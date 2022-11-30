// INCOMPLETE

package frc.robot.BreakerLib.devices.sensors.imu;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADIS16448_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import static edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis.*;

import java.util.function.Supplier;

/**
 * ADIS16448 IMU using BreakerLib interfaces. Calibration is done on
 * construction.
 */
public class BreakerADIS16448 extends BreakerGenericIMU {

    private ADIS16448_IMU imu;
    private Supplier<Double> yawSupplier, pitchSupplier, rollSupplier;
    private Supplier<Double> yawRateSupplier, pitchRateSupplier, rollRateSupplier;

    /**
     * ADIS16448 gyro with Z axis as yaw, MXP SPI port, and calibration time of 512
     * ms.
     */
    public BreakerADIS16448() {
        imu = new ADIS16448_IMU();
        defineAxes(kZ);
        calibrate();
    }

    /**
     * ADIS16448 with custom parameters. All axes are inverted to match the
     * orientations shown in the linked image, regardless of which axis is defined
     * as the yaw.
     * https://wiki.analog.com/_media/first/adis16448_rotation_figure.jpg?w=400&tok=8704ca
     * 
     * @param yawAxis Which axis will be the yaw.
     * @param port    SPI port.
     * @param calTime Calibration time.
     */
    public BreakerADIS16448(IMUAxis yawAxis, SPI.Port port, CalibrationTime calTime) {
        imu = new ADIS16448_IMU(yawAxis, port, calTime);
        defineAxes(yawAxis);
        calibrate();
    }

    /**
     * Defines yaw, pitch, and roll axes based on set yaw axis. Refer to this image:
     * https://wiki.analog.com/_media/first/adis16448_rotation_figure.jpg?w=400&tok=8704ca
     */
    private void defineAxes(IMUAxis yawAxis) {
        switch (yawAxis) {
            case kZ:
                yawSupplier = () -> imu.getGyroAngleZ();
                yawRateSupplier = () -> imu.getGyroRateZ();
                pitchSupplier = () -> imu.getGyroAngleY();
                pitchRateSupplier = () -> imu.getGyroRateY();
                rollSupplier = () -> imu.getGyroAngleX();
                rollRateSupplier = () -> imu.getGyroRateX();
                break;
            case kY:
                yawSupplier = () -> imu.getGyroAngleY();
                yawRateSupplier = () -> imu.getGyroRateY();
                pitchSupplier = () -> -imu.getGyroAngleZ(); // Pitch is inverted.
                pitchRateSupplier = () -> -imu.getGyroRateZ();
                rollSupplier = () -> imu.getGyroAngleX();
                rollRateSupplier = () -> imu.getGyroRateX();
                break;
            case kX:
                yawSupplier = () -> imu.getGyroAngleX();
                yawRateSupplier = () -> imu.getGyroRateX();
                pitchSupplier = () -> imu.getGyroAngleY();
                pitchRateSupplier = () -> imu.getGyroRateY();
                rollSupplier = () -> -imu.getGyroAngleZ();
                rollRateSupplier = () -> -imu.getGyroRateZ();
                break;
        }
    }

    @Override
    public double getYawDegrees() {
        return yawSupplier.get();
    }

    @Override
    public double getPitchDegrees() {
        return pitchSupplier.get();
    }

    @Override
    public double getRollDegrees() {
        return rollSupplier.get();
    }

    @Override
    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getYawDegrees());
    }

    @Override
    public Rotation2d getPitchRotation2d() {
        return Rotation2d.fromDegrees(getPitchDegrees());
    }

    @Override
    public Rotation2d getRollRotation2d() {
        return Rotation2d.fromDegrees(getRollDegrees());
    }

    @Override
    public double getYawRate() {
        return yawRateSupplier.get();
    }

    @Override
    public double getPitchRate() {
        return pitchRateSupplier.get();
    }

    @Override
    public double getRollRate() {
        return rollRateSupplier.get();
    }

    /** Does nothing. */
    public void setYaw(double value) {
    }

    /** Does nothing. */
    public void setPitch(double value) {
    }

    /** Does nothing. */
    public void setRoll(double value) {

    }

    @Override
    public double getRawYaw() {
        return getYawDegrees();
    }

    @Override
    public double getRawPitch() {
        return getPitchDegrees();
    }

    @Override
    public double getRawRoll() {
        return getRollDegrees();
    }

    @Override
    public double getRawYawRate() {
        return getYawRate();
    }

    @Override
    public double getRawPitchRate() {
        return getPitchRate();
    }

    @Override
    public double getRawRollRate() {
        return getRollRate();
    }

    @Override
    public double[] getRawAngles() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double[] getRawGyroRates() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Quaternion getQuaternion() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Rotation3d getRotation3d() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Rotation3d getRawRotation3d() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void calibrate() {
        imu.calibrate();
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub

    }

    @Override
    public double[] getRawAccelerometerVals() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getRawAccelX() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRawAccelY() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRawAccelZ() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void setRange(Range range) {
        // TODO Auto-generated method stub

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
    public void runSelfTest() {
        // TODO Auto-generated method stub

    }
}
