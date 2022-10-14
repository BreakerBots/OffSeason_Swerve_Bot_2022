package frc.robot.BreakerLib.devices.sensors.imu.kuailabs;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.BreakerLib.devices.sensors.BreakerGenericMagnetometer;
import frc.robot.BreakerLib.devices.sensors.imu.BreakerGenericIMU;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.BreakerLib.position.geometry.BreakerRotation3d;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;


/** Breaker NavX gyro. Calibration must be called manually. */
public class BreakerAHRS extends BreakerGenericIMU implements BreakerGenericMagnetometer {

    private AHRS imu;

    public BreakerAHRS() {
        imu = new AHRS();
        deviceName = " NavX_AHRS_IMU (SPI: default) ";
    }

    public BreakerAHRS(SPI.Port spi_port_id) {
        imu = new AHRS(spi_port_id);
        deviceName = " NavX_AHRS_IMU (SPI: "+spi_port_id.toString()+") ";
    }

    public BreakerAHRS(SPI.Port spi_port_id, byte update_rate_hz) {
        imu = new AHRS(spi_port_id, update_rate_hz);
        deviceName = " NavX_AHRS_IMU (SPI: "+spi_port_id.toString()+") ";
    }

    public BreakerAHRS(SPI.Port spi_port_id, int spi_bitrate, byte update_rate_hz) {
        imu = new AHRS(spi_port_id, spi_bitrate, update_rate_hz);
        deviceName = " NavX_AHRS_IMU (SPI: "+spi_port_id.toString()+") ";
    }

    public BreakerAHRS(I2C.Port i2c_port_id) {
        imu = new AHRS(i2c_port_id);
        deviceName = " NavX_AHRS_IMU (I2C: "+i2c_port_id.toString()+") ";
    }

    public BreakerAHRS(I2C.Port i2c_port_id, byte update_rate_hz) {
        imu = new AHRS(i2c_port_id, update_rate_hz);
        deviceName = " NavX_AHRS_IMU (I2C: "+i2c_port_id.toString()+") ";
    }

    public BreakerAHRS(SerialPort.Port serial_port_id) {
        imu = new AHRS(serial_port_id);
        deviceName = " NavX_AHRS_IMU (Serial: "+serial_port_id.toString()+") ";
    }

    public BreakerAHRS(SerialPort.Port serial_port_id, SerialDataType data_type, byte update_rate_hz) {
        imu = new AHRS(serial_port_id, data_type, update_rate_hz);
        deviceName = " NavX_AHRS_IMU (Serial: "+serial_port_id.toString()+") ";
    }

    @Override
    public double getPitchDegrees() {
        return BreakerMath.angleModulus(imu.getPitch());
    }

    @Override
    public double getYawDegrees() {
        return BreakerMath.angleModulus(imu.getYaw());
    }

    @Override
    public double getRollDegrees() {
        return BreakerMath.angleModulus(imu.getRoll());
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
        return new double[] { getPitchDegrees(), getYawDegrees(), getRollDegrees() };
    }

    @Override
    public double getRawYaw() {
        return imu.getAngle();
    }

    /** Does nothing. */
    @Override
    public double getRawPitch() {
        return getPitchDegrees();
    }

    /** Does nothing. */
    @Override
    public double getRawRoll() {
        return getRollDegrees();
    }

    /** Resets yaw to 0 */
    @Override
    public void reset() {
        imu.reset();
    }

    /**
     * Calibrates the IMU. Do this when the robot won't be moving, like when the
     * robot is turned on.
     */
    public void calibrate() {
        imu.calibrate();
    }

    /** Does nothing. */
    @Override
    public void setPitch(double value) {
    }

    /** Does nothing. */
    @Override
    public void setYaw(double value) {
    }

    /** Does nothing. */
    @Override
    public void setRoll(double value) {
    }

    @Override
    public double[] getRawGyroRates() {
        return new double[] {imu.getRawGyroX(), imu.getRawGyroY(), imu.getRawGyroZ()};
    }

    @Override
    public double getRawPitchRate() {
        return imu.getRawGyroX();
    }

    @Override
    public double getPitchRate() {
        return getRawPitchRate();
    }

    @Override
    public double getRawYawRate() {
        return imu.getRawGyroY();
    }

    @Override
    public double getYawRate() {
        return getRawYawRate();
    }

    @Override
    public double getRawRollRate() {
        return imu.getRawGyroZ();
    }

    @Override
    public double getRollRate() {
        return getRawRollRate();
    }

    @Override
    public BreakerRotation3d getRawRotation3d() {
        return new BreakerRotation3d(getPitchRotation2d(), getYawRotation2d(), getRollRotation2d());
    }

    @Override
    public double[] getRawAccelerometerVals() {
        return new double[] {getRawAccelX(), getRawAccelY(), getRawAccelZ()};
    }

    @Override
    public double getRawAccelX() {
        return imu.getRawAccelX();
    }

    @Override
    public double getRawAccelY() {
        return imu.getRawAccelY();
    }

    @Override
    public double getRawAccelZ() {
        return imu.getRawAccelZ();
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
        health = DeviceHealth.NOMINAL;
        faultStr = null;
        if (!imu.isConnected()) {
            health = DeviceHealth.INOPERABLE;
            faultStr += " NOT_CONNECTED ";
        }
        if (imu.isMagneticDisturbance()) {
            health = DeviceHealth.FAULT;
            faultStr += " MAG_DISTERBANCE ";
        }
        if (imu.getTempC() >= 40.0) {
            health = DeviceHealth.FAULT;
            faultStr += " EXTREME_TEMP ";
        }
    }

    @Override
    public double[] getRawFieldStrenghts() {
        double[] strens = {imu.getRawMagX(), imu.getRawMagY(), imu.getRawMagZ()};
        return strens;
    }

    @Override
    public double[] getBiasedFieldStrenghts() {
        BreakerVector3 fieldVec = new BreakerVector3(imu.getRawMagX(), imu.getRawMagY(), imu.getRawMagZ());
        return fieldVec.rotate(getRotation3d()).getInterpolatableData();
    }

    @Override
    public double getCompassFieldStrength() {
        return new BreakerVector3(imu.getRawMagX(), imu.getRawMagY(), imu.getRawMagZ()).getMagnitude();
    }

    @Override
    public double getCompassHeading() {
        return MathUtil.angleModulus(imu.getCompassHeading());
    }

    @Override
    public double getRawCompassHeading() {
        return imu.getCompassHeading();
    }

}
