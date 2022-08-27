// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** BreakerLib encapsulation of an auto path. */
public class BreakerAutoPath {

    private SequentialCommandGroup autoPath;
    private String pathName;

    /** Creates a BreakerAutoPath.
     * 
     * @param pathName Name of the path.
     * @param autoPath Autopath command.
     */
    public BreakerAutoPath(String pathName, SequentialCommandGroup autoPath) {
        this.autoPath = autoPath;
        this.pathName = pathName;
    }

    public String getPathName() {
        return pathName;
    }

    public SequentialCommandGroup getBaseCommandGroup() {
        return autoPath;
    }

    /** Schedules auto path. */
    public SequentialCommandGroup startPath() {
        autoPath.schedule();
        return autoPath;
    }
}
