// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.diff;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.BreakerLib.auto.trajectory.BreakerGenericTrajectoryFollower;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalCommand;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDrive;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/** OUR version of a differential drive ramsete command. */
public class BreakerRamsete extends CommandBase implements BreakerGenericTrajectoryFollower {

    private RamseteCommand ramsete;
    private RamseteController ramseteController;
    private BreakerDiffDrive drivetrain;
    private BreakerGenericOdometer odometer;
    private double currentTimestamp = Timer.getFPGATimestamp();
    private final double startTimestamp = Timer.getFPGATimestamp();
    private double currentTimeSeconds = 0.0;
    private double totalTimeSeconds = 0;
    private boolean stopAtEnd;
    private Trajectory trajectoryToFollow;
    private List<BreakerConditionalCommand> attachedConditionalCommands;

    /**
     * Constructor for BreakerRamsete controller using the drivetrain's builtin
     * odometer.
     * 
     * @param trajectoryToFollow    Self-explanatory.
     * @param drivetrain            Differential drivetrain to use.
     * @param ramseteB              Proportional constant for ramsete.
     * @param ramseteZeta           Damping constant for ramsete.
     * @param stopAtEnd             Whether the robot stops on completion.
     * @param subsystemRequirements Subsystems required for this path.
     */
    public BreakerRamsete(BreakerTrajectoryPath trajectoryPath, BreakerDiffDrive drivetrain, double ramseteB,
            double ramseteZeta, boolean stopAtEnd, Subsystem... subsystemRequirements) {
        this.drivetrain = drivetrain;
        this.odometer = drivetrain;
        trajectoryToFollow = trajectoryPath.getBaseTrajectory();

        BreakerLog.logBreakerLibEvent("BreakerRamsete command instance has started, total cumulative path time: "
                + trajectoryToFollow.getTotalTimeSeconds());

        ramseteController = new RamseteController(ramseteB, ramseteZeta);
        ramsete = new RamseteCommand(trajectoryToFollow, odometer::getOdometryPoseMeters, ramseteController,
                drivetrain.getFeedforward(),
                drivetrain.getKinematics(), drivetrain::getWheelSpeeds, drivetrain.getLeftPIDController(),
                drivetrain.getRightPIDController(), drivetrain::tankDriveVoltage, subsystemRequirements);
        ramsete.schedule();

        totalTimeSeconds = trajectoryToFollow.getTotalTimeSeconds();
        this.stopAtEnd = stopAtEnd;
        attachedConditionalCommands = new ArrayList<>();

        try {
            attachedConditionalCommands.addAll(trajectoryPath.getAttachedConditionalCommands());
        } catch (Exception e) {
            BreakerLog.logError(e.toString());
        }
    }

    /**
     * Constructor for BreakerRamsete controller using an external odometer.
     * 
     * @param trajectoryToFollow    Self-explanatory.
     * @param drivetrain            Differential drivetrain to use.
     * @param odometer              Odometer to use, especially if you want to use a
     *                              vision-based odometer.
     * @param ramseteB              Proportional constant for ramsete.
     * @param ramseteZeta           Damping constant for ramsete.
     * @param stopAtEnd             Whether the robot stops on completion.
     * @param subsystemRequirements Subsystems required for this path.
     */
    public BreakerRamsete(BreakerTrajectoryPath trajectoryPath, BreakerDiffDrive drivetrain,
            BreakerGenericOdometer odometer, double ramseteB,
            double ramseteZeta, boolean stopAtEnd, Subsystem... subsystemRequirements) {

        this.drivetrain = drivetrain;
        this.odometer = odometer;
        trajectoryToFollow = trajectoryPath.getBaseTrajectory();

        BreakerLog.logBreakerLibEvent("BreakerRamsete command instance has started, total cumulative path time: "
                + trajectoryToFollow.getTotalTimeSeconds());

        // Ramsete command is constructed.
        ramseteController = new RamseteController(ramseteB, ramseteZeta);
        ramsete = new RamseteCommand(trajectoryToFollow, odometer::getOdometryPoseMeters, ramseteController,
                drivetrain.getFeedforward(),
                drivetrain.getKinematics(), drivetrain::getWheelSpeeds, drivetrain.getLeftPIDController(),
                drivetrain.getRightPIDController(), drivetrain::tankDriveVoltage, subsystemRequirements);
        ramsete.schedule();

        totalTimeSeconds = trajectoryToFollow.getTotalTimeSeconds();
        this.stopAtEnd = stopAtEnd;
        attachedConditionalCommands = new ArrayList<>();

        // Attempts to add attached conditional commands.
        try {
            attachedConditionalCommands.addAll(trajectoryPath.getAttachedConditionalCommands());
        } catch (Exception e) {
            BreakerLog.logError(e.toString());
        }
    }

    /** Calculates elapsed time. */
    private void calculateTime() {
        currentTimestamp = Timer.getFPGATimestamp();
        currentTimeSeconds = currentTimestamp - startTimestamp; // formerly startTimestamp - currentTimestamp
    }

    @Override
    public void execute() {
        calculateTime();
        checkAttachedCommands();
    }

    @Override
    public void end(boolean interrupted) {
        if (stopAtEnd) {
            drivetrain.arcadeDrive(0, 0);
        }
        BreakerLog.logBreakerLibEvent("BreakerRamsete command instance has ended");
    }

    @Override
    public boolean isFinished() {
        return ramsete.isFinished();
    }

    @Override
    public Trajectory getCurrentTrajectory() {
        return trajectoryToFollow;
    }

    @Override
    public Trajectory[] getAllTrajectories() {
        Trajectory[] traject = new Trajectory[1];
        traject[0] = trajectoryToFollow;
        return traject;
    }

    @Override
    public double getTotalPathTimeSeconds() {
        return totalTimeSeconds;
    }

    @Override
    public double getCurrentPathTimeSeconds() {
        return currentTimeSeconds;
    }

    @Override
    public boolean getPathStopsAtEnd() {
        return stopAtEnd;
    }

    @Override
    public List<State> getAllStates() {
        return trajectoryToFollow.getStates();
    }

    @Override
    public State getCurrentState() {
        return trajectoryToFollow.sample(getCurrentPathTimeSeconds());
    }

    @Override
    public void attachConditionalCommands(BreakerConditionalCommand... conditionalCommands) {
        for (BreakerConditionalCommand com : conditionalCommands) {
            attachedConditionalCommands.add(com);
        }
    }

    private void checkAttachedCommands() {
        try {
            Iterator<BreakerConditionalCommand> iterator = attachedConditionalCommands.iterator();
            while (iterator.hasNext()) {
                BreakerConditionalCommand com = iterator.next();
                if (com.checkCondition(currentTimeSeconds, odometer.getOdometryPoseMeters())) {
                    com.startRunning();
                    iterator.remove();
                }
            }
        } catch (Exception e) {
        }
    }

}
