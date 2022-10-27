// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.selftest;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.fasterxml.jackson.databind.JsonNode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerSounds;
import frc.robot.BreakerLib.util.BreakerJSONUtil;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class SelfTest extends SubsystemBase {
  /** Creates a new SelfTest. */
  private int cycleCount;
  private static String lastSystemCheck;
  private static List<BreakerSelfTestable> devices = new ArrayList<BreakerSelfTestable>();
  private static int cyclesbetweenPerSelfCecks = 250;
  private static boolean lastCheckPassed = true;
  private static BreakerFalconOrchestra orchestra;
  private static boolean usesOrchestra = false;
  private static boolean autoRegesterDevices = true;
  private static boolean selfTestEnabled = true;

  public SelfTest(double secondsBetweenPeriodicSelfChecks) {
    SelfTest.cyclesbetweenPerSelfCecks = (int) (secondsBetweenPeriodicSelfChecks * 50);
    SelfTest.usesOrchestra = false;
    SelfTest.autoRegesterDevices = true;
  }

  public SelfTest(double secondsBetweenPeriodicSelfChecks, boolean autoRegesterDevices) {
    SelfTest.cyclesbetweenPerSelfCecks = (int) (secondsBetweenPeriodicSelfChecks * 50);
    SelfTest.usesOrchestra = false;
    SelfTest.autoRegesterDevices = autoRegesterDevices;
  }

  public SelfTest(double secondsBetweenPeriodicSelfChecks, BreakerFalconOrchestra orchestra) {
    SelfTest.cyclesbetweenPerSelfCecks = (int) (secondsBetweenPeriodicSelfChecks * 50);
    SelfTest.orchestra = orchestra;
    SelfTest.usesOrchestra = true;
  }

  public SelfTest(double secondsBetweenPeriodicSelfChecks, BreakerFalconOrchestra orchestra, boolean autoRegesterDevices) {
    SelfTest.cyclesbetweenPerSelfCecks = (int) (secondsBetweenPeriodicSelfChecks * 50);
    SelfTest.orchestra = orchestra;
    SelfTest.usesOrchestra = true;
    SelfTest.autoRegesterDevices = autoRegesterDevices;
  }

  public static void setSelfTestEnabled(boolean isEnabled) {
    selfTestEnabled = isEnabled;
  }

  public static boolean isSelfTestEnabled() {
      return selfTestEnabled;
  }

  public static void autoRegisterDevice(BreakerSelfTestable device) {
    if (autoRegesterDevices) {
      devices.add(device);
    }
  }

  public static void autoRegesterDevices(BreakerSelfTestable... devices) {
    if (autoRegesterDevices) {
      addDevices(devices);
    }
  }


  private static void runAlarm() {
    if (usesOrchestra) {
      orchestra.startLoopedSong(BreakerSounds.GeneralAlarmSound);
    }
  }

  public static void addDevice(BreakerSelfTestable device) {
    devices.add(device);
  }

  public static void addDevices(BreakerSelfTestable... devicesToAdd) {
    for (BreakerSelfTestable div: devicesToAdd) {
      devices.add(div);
    }
  }

  public static String getLastSelfCheck() {
    return lastSystemCheck;
  }

  public static boolean getLastSelfCheckPassed() {
    return lastCheckPassed;
  }

  public static boolean getAutoRegesterDevicesIsEnabled() {
    return autoRegesterDevices;
  }

  public static void runSelfCheck() {
    StringBuilder work = new StringBuilder("\n RUNNING SELF CHECK: \n");
    List<BreakerSelfTestable> faultDevices = new ArrayList<BreakerSelfTestable>();
    for (BreakerSelfTestable device: devices) {
      device.runSelfTest();
      if (device.hasFault()) {
        faultDevices.add(device);
      }
    }
    if (!faultDevices.isEmpty()) {
      work.append(" SELF CHECK FAILED - FAULTS FOUND: \n");
      lastCheckPassed = false;
      for (BreakerSelfTestable faultDiv: faultDevices) {
        work.append(" | " + faultDiv.getDeviceName() + "-" + faultDiv.getFaults() + " | ");
      }
      runAlarm();
    } else {
      work.append(" SELF CHECK PASSED ");
      lastCheckPassed = true;
    }
    lastSystemCheck = work.toString();
    BreakerLog.log(lastSystemCheck);
  }

  @Override
  public void periodic() {
    if ((cycleCount ++ % cyclesbetweenPerSelfCecks == 0) && selfTestEnabled) {
      runSelfCheck();
    }
  }
}
