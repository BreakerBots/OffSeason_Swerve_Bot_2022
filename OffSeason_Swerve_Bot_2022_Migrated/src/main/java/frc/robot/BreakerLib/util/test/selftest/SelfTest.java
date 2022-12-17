// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.selftest;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerSounds;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/**
 * Core class of BreakerLib's SelfTest fuctionality, handling periodic passive
 * (unless user configured otherwise)
 * diagnostic self tests of all manually or automatically regeistered
 * {@link BreakerSelfTestable} compatible devices, or devices registered
 * through an instance of the {@link SystemDiagnostics} class.
 */
public class SelfTest{

  private static int cycleCount;
  private static String lastSystemTest;
  private static List<BreakerSelfTestable> deviceList = new ArrayList<BreakerSelfTestable>();

  /** # of cycles between self-tests. For accurate conversion, seconds must be divisible by time per cycle (usually 20ms) */
  private static int cyclesBetweenSelfTests = 250;
  private static boolean lastTestPassed = true;
  private static BreakerFalconOrchestra orchestra;
  private static boolean usesOrchestra = false;
  private static boolean autoRegisterDevices = true;
  private static boolean selfTestEnabled = true;

  /** Runs self-test loop only when enough time has passed. Otherwise schedules a null command. */
  private static ConditionalCommand selfTestLoop = new ConditionalCommand(new InstantCommand(() -> runSelfTest()), new WaitCommand(0), () -> enoughCyclesPassed());

  /**
   * Configures and enables a SelfTest cycle. Devices automatically are
   * registered.
   * 
   * @param secondsBetweenSelfTests # of seconds between self-tests.
   */
  public SelfTest(double secondsBetweenSelfTests) {
    SelfTest.cyclesBetweenSelfTests = (int) (secondsBetweenSelfTests * 50);
    SelfTest.usesOrchestra = false;
    SelfTest.autoRegisterDevices = true;
    startPeriodicTests();
  }

  /**
   * Configures and enables a SelfTest cycle.
   * 
   * @param secondsBetweenPeriodicSelftests # of seconds between self-tests.
   * @param autoRegisterDevices              Whether or not devices will
   *                                         automatically register.
   */
  public SelfTest(double secondsBetweenPeriodicSelftests, boolean autoRegisterDevices) {
    SelfTest.cyclesBetweenSelfTests = (int) (secondsBetweenPeriodicSelftests * 50);
    SelfTest.usesOrchestra = false;
    SelfTest.autoRegisterDevices = autoRegisterDevices;
    startPeriodicTests();
  }

  /**
   * Configures and enables a SelfTest cycle. Devices automatically are
   * registered.
   * 
   * @param secondsBetweenPeriodicSelftests # of seconds between self-tests.
   * @param orchestra                        Falcon Orchestra to use.
   */
  public SelfTest(double secondsBetweenPeriodicSelftests, BreakerFalconOrchestra orchestra) {
    SelfTest.cyclesBetweenSelfTests = (int) (secondsBetweenPeriodicSelftests * 50);
    SelfTest.orchestra = orchestra;
    SelfTest.usesOrchestra = true;
    startPeriodicTests();
  }

  /**
   * Full constructor for SelfTest.
   * 
   * @param secondsBetweenPeriodicSelftests # of seconds between self-tests.
   * @param orchestra                        Falcon Orchestra to use.
   * @param autoRegisterDevices              Whether or not devices will
   *                                         automatically register.
   */
  public SelfTest(double secondsBetweenPeriodicSelftests, BreakerFalconOrchestra orchestra,
      boolean autoRegisterDevices) {
    SelfTest.cyclesBetweenSelfTests = (int) (secondsBetweenPeriodicSelftests * 50);
    SelfTest.orchestra = orchestra;
    SelfTest.usesOrchestra = true;
    SelfTest.autoRegisterDevices = autoRegisterDevices;
    startPeriodicTests();
  }

  /** Schedules the command that will run the self-test loop. */
  public static void startPeriodicTests() {
    CommandScheduler.getInstance().schedule(selfTestLoop);
  }

  /** @return If enough cycles have passed to run another self-test. */
  public static boolean enoughCyclesPassed() {
    return cycleCount++ % cyclesBetweenSelfTests == 0;
  }

  /** Enables or disables the periodic self test diagnstic test cycle. */
  public static void setSelfTestEnabled(boolean isEnabled) {
    selfTestEnabled = isEnabled;
  }

  /** @return If the periodic self test diagnstic test cycle is enabled. */
  public static boolean isSelfTestEnabled() {
    return selfTestEnabled;
  }

  /**
   * Automatically adds given {@link BreakerSelfTestable} compatible devices to
   * the SelfTest queue if automatic registration is enabled.
   * <p>
   * <b>
   * WARNING: should only be used in BreakerLib internal classes and is toggled
   * based on user config of Self-Test.
   * <b>
   * 
   * @param device Device to add.
   */
  public static void autoRegisterDevices(BreakerSelfTestable device) {
    if (autoRegisterDevices) {
      addDevices(device);
    }
  }

  /**
   * Automatically adds given {@link BreakerSelfTestable} compatible devices to
   * the SelfTest queue if automatic registration is enabled.
   * <p>
   * <b>
   * WARNING: This should only be used in BreakerLib internal classes and is
   * toggled
   * based on user config of Self-Test.
   * <b>
   * 
   * @param devices Devices to automatically be added.
   */
  protected static void autoRegisterDevices(BreakerSelfTestable... devices) {
    if (autoRegisterDevices) {
      addDevices(devices);
    }
  }

  /** If orchestra is in use, alarm will play. */
  private static void runAlarm() {
    if (usesOrchestra) {
      orchestra.startLoopedSong(BreakerSounds.GeneralAlarmSound);
    }
  }

  /**
   * Manually adds given {@link BreakerSelfTestable} compatible devices to the
   * SelfTest queue.
   * <p>
   * <b>
   * WARNING: If this is done to a device that has allready been added
   * (including automatic additions, if enabled), will be tested twice,
   * possably with signifcant runtime expence
   * <b>
   * 
   * @param devicesToAdd Devices to be added to queue.
   */
  public static void addDevices(BreakerSelfTestable... devicesToAdd) {
    for (BreakerSelfTestable device : devicesToAdd) {
      deviceList.add(device);
    }
  }

  /**
   * @return The logged fault string result of the most recent self test cycle.
   */
  public static String getLastSelftest() {
    return lastSystemTest;
  }

  /**
   * @return True if the last self test passed without fault, false
   *         otherwise.
   */
  public static boolean lastSelfTestPassed() {
    return lastTestPassed;
  }

  /**
   * @return True if {@link SelfTest#autoRegisterDevice()} and
   *         {@link SelfTest#autoRegisterDevices()} are enabled by the user, false
   *         otherwise.
   */
  public static boolean getAutoRegesterDevicesIsEnabled() {
    return autoRegisterDevices;
  }

  /**
   * Self-Test loop method. Automatically called periodically when SelfTest object is constructed.
   */
  public static void runSelfTest() {
    StringBuilder work = new StringBuilder("\n RUNNING SELF test: \n");
    List<BreakerSelfTestable> faultDevices = new ArrayList<BreakerSelfTestable>();

    for (BreakerSelfTestable device : deviceList) {
      device.runSelfTest();
      if (device.hasFault()) {
        faultDevices.add(device);
      }
    }

    if (!faultDevices.isEmpty()) {
      work.append(" SELF test FAILED - FAULTS FOUND: \n");
      lastTestPassed = false;
      for (BreakerSelfTestable faultyDevice : faultDevices) {
        work.append(" | " + faultyDevice.getDeviceName() + "-" + faultyDevice.getFaults() + " | ");
      }
      runAlarm(); // Only works if Orchestra is enabled.
    } else {
      work.append(" SELF test PASSED ");
      lastTestPassed = true;
    }
    lastSystemTest = work.toString();
    BreakerLog.log(lastSystemTest);
  }
}
