// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.BreakerLib.subsystem.cores.shooter;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

// import edu.wpi.first.math.Pair;
// import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
// import frc.robot.BreakerLib.util.power.DevicePowerMode;
// import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
// import frc.robot.BreakerLib.util.test.suites.flywheelSuite.BreakerFlywheelTestSuite;
// import frc.robot.BreakerLib.util.vendorutil.BreakerREVUtil;

// /** A class representing a robot's shooter flywheel and its assocated controle loop */
// public class BreakerNeoFlywheel extends BreakerGenericFlywheel {
//     private CANSparkMax lFlyMotor;
//     private CANSparkMax[] motors;
    

//     public BreakerNeoFlywheel(BreakerFlywheelConfig config, CANSparkMax... flywheelMotors) {
//         super(config);

//         lFlyMotor = flywheelMotors[0];
//         motors = flywheelMotors;

//         SparkMaxPIDController drivePID = lFlyMotor.getPIDController();
//         drivePID.setP(config.getkP());
//         drivePID.setI(config.getkI());
//         drivePID.setD(config.getkD());
//         drivePID.setFF(config.getkF());
        
//         for (int i = 1; i < motors.length; i++) {
//             motors[i].follow(lFlyMotor);
//         }
//     }

//     @Override
//     public double getFlywheelVelRSU() {
//         return lFlyMotor.getEncoder().getVelocity();
//     }

//     @Override
//     public double getFlywheelRPM() {
//         return getFlywheelVelRSU();
//     }

//     @Override
//     protected void runFlywheel() {
//         double flySetSpd = flywheelTargetRPM * config.getFlywheelGearRatio();
//         double feedforward = ffProvider.getArbitraryFeedforwardValue(flywheelTargetRPM);
//         lFlyMotor.getPIDController().setReference(flySetSpd, CANSparkMax.ControlType.kVelocity , 0, feedforward, ArbFFUnits.kPercentOut);
//     }

//     @Override
//     public BreakerFlywheelTestSuite getTestSuite() {
//         return testSuite;
//     }

//     @Override
//     public void runSelfTest() {
//        faultStr = "";
//        health = DeviceHealth.NOMINAL;
//        for (CANSparkMax mot: motors) {
//            short motFaults = mot.getFaults();
//            Pair<DeviceHealth, String> faultData = BreakerREVUtil.getSparkMaxHealthAndFaults(motFaults);
//            if (faultData.getFirst() != DeviceHealth.NOMINAL) {
//                faultStr += faultData.getSecond();
//                health = health != DeviceHealth.NOMINAL ? faultData.getFirst() : health;
//            }
//        }
//     }

//     @Override
//     public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig, double... managementPerameters) {
//         // TODO Auto-generated method stub
//         return null;
//     }

//     @Override
//     public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
//         // TODO Auto-generated method stub
        
//     }

//     @Override
//     public void returnToAutomaticPowerManagement() {
//         // TODO Auto-generated method stub
        
//     }

//     @Override
//     public boolean isUnderAutomaticControl() {
//         // TODO Auto-generated method stub
//         return false;
//     }

//     @Override
//     public DevicePowerMode getPowerMode() {
//         // TODO Auto-generated method stub
//         return null;
//     }
// }
