// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.auto.trajectory.management;

import BreakerLib.position.odometry.BreakerGenericOdometer;
import BreakerLib.util.logging.BreakerLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class BreakerStartTrajectoryPath extends InstantCommand {
  private BreakerGenericOdometer odometryProvider;
  private Pose2d startingPose;

  
  public BreakerStartTrajectoryPath(BreakerGenericOdometer odometryProvider, Pose2d startingPose) {
    this.odometryProvider = odometryProvider;
    this.startingPose = startingPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BreakerLog.logBreakerLibEvent("A Trajectory Path has been started - A BreakerStartTrajectoryPath instance has been created and ran");
    odometryProvider.setOdometryPosition(startingPose);
  }
}
