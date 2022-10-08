package frc.robot.BreakerLib.devices.sensors.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.BreakerLib.position.geometry.BreakerRotation3d;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

/** Breaker NavX gyro. Calibration must be called manually. */
public class BreakerAHRS extends BreakerGenericIMU{

    private AHRS imu;

    public BreakerAHRS() {
        imu = new AHRS();
    }

    public BreakerAHRS(SPI.Port spi_port_id) {
        imu = new AHRS(spi_port_id);
    }

    public BreakerAHRS(SPI.Port spi_port_id, byte update_rate_hz) {
        imu = new AHRS(spi_port_id, update_rate_hz);
    }

    public BreakerAHRS(SPI.Port spi_port_id, int spi_bitrate, byte update_rate_hz) {
        imu = new AHRS(spi_port_id, spi_bitrate, update_rate_hz);
    }

    public BreakerAHRS(I2C.Port i2c_port_id) {
        imu = new AHRS(i2c_port_id);
    }

    public BreakerAHRS(I2C.Port i2c_port_id, byte update_rate_hz) {
        imu = new AHRS(i2c_port_id, update_rate_hz);
    }

    public BreakerAHRS(SerialPort.Port serial_port_id) {
        imu = new AHRS(serial_port_id);
    }

    public BreakerAHRS(SerialPort.Port serial_port_id, SerialDataType data_type, byte update_rate_hz) {
        imu = new AHRS(serial_port_id, data_type, update_rate_hz);
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
        return new double[] {getPitchDegrees(), getYawDegrees(), getRollDegrees()};
    }

    /** Resets yaw to 0 */
    @Override
    public void reset() {
        imu.reset();
    }

    /** Calibrates the IMU. Do this when the robot won't be moving, like when the robot is turned on. */
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
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getPitchRate() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getYawRate() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRollRate() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public BreakerRotation3d getRawRotation3d() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public short[] getRawAccelerometerVals() {
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
