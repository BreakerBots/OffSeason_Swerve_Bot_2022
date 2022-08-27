// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory;

import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalCommand;

/** Trajectory-following interface for both differential and swerve drive. */
public interface BreakerGenericTrajectoryFollower {

    /** Returns the currently followed trajectory. */
    public abstract Trajectory getCurrentTrajectory();

    /** Returns all trajectgories followed by this path. */
    public abstract Trajectory[] getAllTrajectories();

    /** Total duration of path in seconds. */
    public abstract double getTotalPathTimeSeconds();

    /** Duration of current path in seconds. */
    public abstract double getCurrentPathTimeSeconds();

    /** If the path stops once completed. */
    public abstract boolean getPathStopsAtEnd();

    /** Get all trajectory states. */
    public abstract List<State> getAllStates();

    /** Gets current state of trajectory. */
    public abstract State getCurrentState();

    /**
     * Attaches conditional commands to path.
     * 
     * @param conditionalCommands Conditional commands to attach.
     */
    public abstract void attachConditionalCommands(BreakerConditionalCommand... conditionalCommands);

}
