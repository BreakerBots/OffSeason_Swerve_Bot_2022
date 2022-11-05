// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalEvent;

/** Represents a trajectory and its optionaly attached commands */
public class BreakerTrajectoryPath {
    private List<BreakerConditionalEvent> attachedConditionalEvents = new ArrayList<>();
    private Trajectory baseTrajectory;
    private boolean stopAtEnd;

    public BreakerTrajectoryPath(Trajectory baseTrajectory) {
        this(baseTrajectory, true);
    }

    public BreakerTrajectoryPath(Trajectory baseTrajectory, BreakerConditionalEvent... conditionalEvents) {
        this(baseTrajectory, true, conditionalEvents);
    }

    public BreakerTrajectoryPath(Trajectory baseTrajectory, boolean stopAtEnd) {
        this.baseTrajectory = baseTrajectory;
        this.stopAtEnd = stopAtEnd;
        attachedConditionalEvents= new ArrayList<>();
    }

    public BreakerTrajectoryPath(Trajectory baseTrajectory, boolean stopAtEnd, BreakerConditionalEvent... conditionalEvents) {
        this.baseTrajectory = baseTrajectory;
        this.stopAtEnd = stopAtEnd;
        for (BreakerConditionalEvent com: conditionalEvents) {
            attachedConditionalEvents.add(com);
        }
    }

    public List<BreakerConditionalEvent> getAttachedConditionalEvents() {
        return attachedConditionalEvents;
    }

    public Trajectory getBaseTrajectory() {
        return baseTrajectory;
    }

    public boolean stopAtEnd() {
        return stopAtEnd;
    }
}
