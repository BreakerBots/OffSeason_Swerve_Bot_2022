// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve.ltv;

import java.util.ArrayList;
import java.util.Iterator;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.auto.trajectory.BreakerGenericAutoPathFollower;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalEvent;
import frc.robot.BreakerLib.auto.trajectory.swerve.rotation.BreakerGenericSwerveRotationSupplier;
import frc.robot.BreakerLib.auto.trajectory.swerve.rotation.BreakerSwerveRotationSupplier;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/** Add your docs here. */
public class BreakerLTVUnicycleSwerveAutoPathFollower extends CommandBase implements BreakerGenericAutoPathFollower  {
    private final Timer timer = new Timer();
    private BreakerLTVUnicycleSwerveAutoPathFollowerConfig config;
    private BreakerTrajectoryPath trajectoryPath;
    private BreakerGenericSwerveRotationSupplier rotationSupplier;
    private ArrayList<BreakerConditionalEvent> remainingEvents;
   /** Creates a new {@link BBreakerLTVUnicycleSwerveAutoPathFollower} that follows the given {@link BreakerTrajectoryPath}
   * <br><br>NOTE: Robot's rotation setpoint defaults to th trajectory's lookahead point
   * @param config The {@link BreakerSwerveAutoPathFollowerConfig} instnace that defignes this {@link BBreakerLTVUnicycleSwerveAutoPathFollower} instnaces base setup
   * @param trajectoryPath The {@link BreakerTrajectoryPath} that this {@link BreakerLTVUnicycleSwerveAutoPathFollower} will follow
    */
    public BreakerLTVUnicycleSwerveAutoPathFollower(BreakerLTVUnicycleSwerveAutoPathFollowerConfig config, BreakerTrajectoryPath trajectoryPath) {
        addRequirements(config.getDrivetrain());
        this.config = config;
        this.trajectoryPath = trajectoryPath;
        rotationSupplier = new BreakerSwerveRotationSupplier((Double curTime) -> (trajectoryPath.getBaseTrajectory().sample(curTime).poseMeters.getRotation()));
        remainingEvents = new ArrayList<>(trajectoryPath.getAttachedConditionalEvents());
      }
    
      /** Creates a new {@link BreakerLTVUnicycleSwerveAutoPathFollower} that follows the given {@link BreakerTrajectoryPath}
       * @param config The {@link BreakerLTVUnicycleSwerveAutoPathFollowerConfig} instnace that defignes this {@link BreakerLTVUnicycleSwerveAutoPathFollower} instnaces base setup
       * @param rotationSupplier The {@link BreakerGenericSwerveRotationSupplier} that returns this path follower's rotation setpoint
       * @param trajectoryPath The {@link BreakerTrajectoryPath} that this {@link BreakerSwerveAutoPathFollower} will follow
        */
      public BreakerLTVUnicycleSwerveAutoPathFollower(BreakerLTVUnicycleSwerveAutoPathFollowerConfig config, BreakerGenericSwerveRotationSupplier rotationSupplier, BreakerTrajectoryPath trajectoryPath) {
        addRequirements(config.getDrivetrain());
        this.config = config;
        this.trajectoryPath = trajectoryPath;
        this.rotationSupplier = rotationSupplier;
        remainingEvents = new ArrayList<>();
      }

    
  @Override
  public void initialize() {
    BreakerLog.logBreakerLibEvent("A new BreakerLTVUnicycleSwerveAutoPathFollower instance has started");
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double curTime = timer.get();

    State desiredState = trajectoryPath.getBaseTrajectory().sample(curTime);

    ChassisSpeeds targetChassisSpeeds =
        config.getUnicycleController().calculate(
            config.getOdometer().getOdometryPoseMeters(),
            new Pose2d(
                desiredState.poseMeters.getTranslation(), 
                rotationSupplier.getRotation(curTime)),
            desiredState.velocityMetersPerSecond, 
            desiredState.velocityMetersPerSecond);

    config.getDrivetrain().move(targetChassisSpeeds, false);

    if (remainingEvents.size() > 0) {
      Iterator<BreakerConditionalEvent> iterator = remainingEvents.iterator();
      while (iterator.hasNext()) {
        BreakerConditionalEvent ev = iterator.next();
        if (ev.checkCondition(timer.get(), config.getOdometer().getOdometryPoseMeters())) {
          ev.getBaseCommand().schedule();
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
    if (interrupted) {
      BreakerLog.logBreakerLibEvent("A BreakerLTVUnicycleSwerveAutoPathFollower instance was interrupted");
    } else {
      BreakerLog.logBreakerLibEvent("A BreakerLTVUnicycleSwerveAutoPathFollower instance has ended normaly");
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
