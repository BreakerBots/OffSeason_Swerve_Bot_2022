// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint;

import java.util.ArrayList;
import java.util.Currency;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.BreakerLib.auto.trajectory.swerve.rotation.BreakerSwerveRotationSupplier;
import frc.robot.BreakerLib.auto.waypoint.BreakerWaypointPath;
import frc.robot.BreakerLib.physics.vector.BreakerVector2;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerSwerveWaypointFollower extends CommandBase {
    private BreakerSwerveWaypointFollowerConfig config;
    private final Timer timer = new Timer();
    private HolonomicDriveController driveController;
    private BreakerWaypointPath waypointPath;
    private BreakerSwerveRotationSupplier rotationSupplier;
    private Translation2d prevWp;
    private ArrayList<Translation2d> waypoints;
    private double totalDistance;
    public BreakerSwerveWaypointFollower(BreakerSwerveWaypointFollowerConfig config, BreakerWaypointPath waypointPath) {
        addRequirements(config.getDrivetrain());
        waypoints = new ArrayList<>();
        for (Translation2d wp: waypointPath.getWaypoints()) {
            waypoints.add(wp);
        }
        prevWp = config.getOdometer().getOdometryPoseMeters().getTranslation();
        totalDistance = waypointPath.getTotalPathDistance() + waypoints.get(0).getDistance(prevWp);
        this.config = config;
        this.waypointPath = waypointPath;
        this.rotationSupplier = new BreakerSwerveRotationSupplier(() -> (BreakerMath.getPointAngleRelativeToOtherPoint(prevWp, waypoints.get(0))));
        driveController = config.getDriveController();
    }

    public BreakerSwerveWaypointFollower(BreakerSwerveWaypointFollowerConfig config, BreakerWaypointPath waypointPath, BreakerSwerveRotationSupplier rotationSupplier) {
        addRequirements(config.getDrivetrain());
        waypoints = new ArrayList<>();
        for (Translation2d wp: waypointPath.getWaypoints()) {
            waypoints.add(wp);
        }
        totalDistance = waypointPath.getTotalPathDistance();
        this.config = config;
        this.waypointPath = waypointPath;
        this.rotationSupplier = rotationSupplier;
        driveController = config.getDriveController();
    }

    public void setWaypointPath(BreakerWaypointPath newWaypointPath) {
        waypointPath = newWaypointPath;
        waypoints.clear();
        for (Translation2d wp: waypointPath.getWaypoints()) {
            waypoints.add(wp);
        }
        totalDistance = waypointPath.getTotalPathDistance();
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    BreakerLog.logBreakerLibEvent("A new BreakerSwerveWaypointFollower instance has started");
    prevWp = config.getOdometer().getOdometryPoseMeters().getTranslation();
    totalDistance +=  + waypoints.get(0).getDistance(prevWp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        Pose2d curPose = config.getOdometer().getOdometryPoseMeters();
        BreakerVector2 curVelVec = config.getOdometer().getMovementState().getDirivitiveFromIndex(0).getLinearForces();
        double curVel = curVelVec.getMagnatude() * (curVelVec.getVectorRotation().getDegrees() >= 0 ? 1 : -1);
        TrapezoidProfile.State curState = new TrapezoidProfile.State(totalDistance - getTotalRemainingDistance(curPose), curVel);
        TrapezoidProfile profile = new TrapezoidProfile(waypointPath.getConstraints(), new TrapezoidProfile.State(totalDistance, 0), curState);
        Rotation2d targetRot = rotationSupplier.getRotation(timer.get());
        ChassisSpeeds targetSpeeds = driveController.calculate(curPose, new Pose2d(waypoints.get(0), targetRot), profile.calculate(0.20).velocity, targetRot);
        config.getDrivetrain().move(ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, config.getOdometer().getOdometryPoseMeters().getRotation()), false);
        if (driveController.atReference()) {
            prevWp = waypoints.remove(0);
        }
  }

  /** @return the internal list that represnets the queue of un-passed waypoints, can be modified */
  public ArrayList<Translation2d> getWaypoints() {
    return waypoints;
  }

  private double getDistanceToWaypoint(Pose2d curPose, Translation2d nextWp) {
    return curPose.getTranslation().getDistance(nextWp);
  }

  private double getTotalRemainingDistance(Pose2d curPose) {
    double totalDist = getDistanceToWaypoint(curPose, waypoints.get(0));
    for (int i = 1; i < waypoints.size(); i++) {
        totalDist += waypoints.get(i-1).getDistance(waypoints.get(i));
    }
    return totalDist;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BreakerLog.logBreakerLibEvent("A BreakerSwerveWaypointFollower instance has ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return waypoints.isEmpty();
  }


    
}
