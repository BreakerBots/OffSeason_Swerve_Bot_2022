// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve;

import java.util.ArrayList;
import java.util.Iterator;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.BreakerLib.auto.trajectory.BreakerGenericAutoPathFollower;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalEvent;
import frc.robot.BreakerLib.auto.trajectory.swerve.rotation.BreakerGenericSwerveRotationSupplier;

public class BreakerSwerveAutoPathFollower extends CommandBase implements BreakerGenericAutoPathFollower {
  /** Creates a new BreakerFollowSwervePath. */
  private Timer timer;
  private BreakerSwerveAutoPathFollowerConfig config;
  private BreakerTrajectoryPath trajectoryPath;
  private BreakerGenericSwerveRotationSupplier rotationSupplier;
  private ArrayList<BreakerConditionalEvent> remainingEvents;
  public BreakerSwerveAutoPathFollower(BreakerSwerveAutoPathFollowerConfig config, BreakerTrajectoryPath trajectoryPath) {
    this.config = config;
    this.trajectoryPath = trajectoryPath;
    remainingEvents = new ArrayList<>(trajectoryPath.getAttachedConditionalEvents());

  }

  public BreakerSwerveAutoPathFollower(BreakerSwerveAutoPathFollowerConfig config, BreakerGenericSwerveRotationSupplier rotationSupplier, BreakerTrajectoryPath trajectoryPath) {
    this.config = config;
    this.trajectoryPath = trajectoryPath;
    remainingEvents = new ArrayList<>();
  }


  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double curTime = timer.get();
    State desiredState = trajectoryPath.getBaseTrajectory().sample(curTime);

    ChassisSpeeds targetChassisSpeeds =
        config.getDriveController().calculate(config.getOdometer().getOdometryPoseMeters(), desiredState, rotationSupplier.getRotation(curTime));

    config.getDrivetrain().move(targetChassisSpeeds, false);

    if (remainingEvents.size() > 0) {
      Iterator<BreakerConditionalEvent> iterator = remainingEvents.iterator();
      while (iterator.hasNext()) {
        BreakerConditionalEvent ev = iterator.next();
        if (ev.checkCondition(timer.get(), config.getOdometer().getOdometryPoseMeters())) {
          ev.getBaseCommand().schedule();;
          iterator.remove();
        }
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    if (trajectoryPath.stopAtEnd()) {
      config.getDrivetrain().stop();
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectoryPath.getBaseTrajectory().getTotalTimeSeconds());
  }

  @Override
  public BreakerTrajectoryPath getTrajectoryPath() {
    return trajectoryPath;
  }

  @Override
  public double getElapsedTimeSeconds() {
    return timer.get();
  }

  @Override
  public boolean getPathStopsAtEnd() {
    return trajectoryPath.stopAtEnd();
  }

  @Override
  public void attachConditionalEvents(BreakerConditionalEvent... conditionalEvents) {
    for (BreakerConditionalEvent ev: conditionalEvents) {
      remainingEvents.add(ev);
    }
  }
}
