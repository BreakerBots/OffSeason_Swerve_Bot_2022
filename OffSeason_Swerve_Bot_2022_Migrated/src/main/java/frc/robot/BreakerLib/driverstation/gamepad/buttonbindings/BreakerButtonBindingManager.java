// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.buttonbindings;

import java.util.ArrayList;
import java.util.function.IntSupplier;

/** Add your docs here. */
public class BreakerButtonBindingManager {
    private static int indexOfMapToUse = 0;
    private static int nextMapIndex = 0;
    public static void addButtonBindingMap(BreakerButtonBindingMap mapToAdd) {
        final int index = nextMapIndex;
        mapToAdd.bindAllConditionaly(() -> indexOfMapToUse == index);
        nextMapIndex++;
    }

    public static int getActiveButtonBindingMapIndex() {
        return indexOfMapToUse;
    }

    public static void setActiveButtonBindingMap(int indexOfMapToUse) {
        BreakerButtonBindingManager.indexOfMapToUse = indexOfMapToUse;
    }
}
