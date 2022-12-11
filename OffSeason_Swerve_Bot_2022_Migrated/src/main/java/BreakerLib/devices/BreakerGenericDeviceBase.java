// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.devices;

import BreakerLib.util.power.BreakerPowerManageable;
import BreakerLib.util.test.selftest.BreakerSelfTestableBase;
import BreakerLib.util.test.selftest.SelfTest;

/** Less complicated alternative to BreakerGenericDevice that also automatically registers devices. */
public abstract class BreakerGenericDeviceBase extends BreakerSelfTestableBase implements BreakerPowerManageable {
    public BreakerGenericDeviceBase() {
        SelfTest.autoRegisterDevice(this);
    }

}
