 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.devices;

import BreakerLib.util.power.BreakerPowerManageable;
import BreakerLib.util.test.selftest.BreakerSelfTestable;

/** Base interface for all Breaker devices. */
public interface BreakerGenericDevice extends BreakerPowerManageable, BreakerSelfTestable {
}
