// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory;

import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalEvent;

/** Trajectory-following interface for both differential and swerve drive. */
public interface BreakerGenericAutoPathFollower {

    /** @return The currently followed trajectory. */
    public abstract BreakerTrajectoryPath getTrajectoryPath();

    /** @return Elapsed path time in seconds. */
    public abstract double getElapsedTimeSeconds();

    /** @return If the path stops once completed. */
    public abstract boolean getPathStopsAtEnd();

    /**
     * Attaches conditional commands to path.
     * 
     * @param conditionalEvents Conditional events to attach.
     */
    public abstract void attachConditionalEvents(BreakerConditionalEvent... conditionalEvents);

}
