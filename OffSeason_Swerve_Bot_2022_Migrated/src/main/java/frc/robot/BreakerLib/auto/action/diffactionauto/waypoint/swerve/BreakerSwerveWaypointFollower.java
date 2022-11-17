// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.action.diffactionauto.waypoint.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.BreakerLib.auto.action.diffactionauto.waypoint.BreakerWaypointPath;
import frc.robot.BreakerLib.auto.trajectory.swerve.rotation.BreakerSwerveRotationSupplier;
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
    public BreakerSwerveWaypointFollower(BreakerSwerveWaypointFollowerConfig config, BreakerWaypointPath waypointPath) {
        waypoints = new ArrayList<>();
        for (Translation2d wp: waypointPath.getWaypoints()) {
            waypoints.add(wp);
        }
        this.config = config;
        this.waypointPath = waypointPath;
        prevWp = new  Translation2d();
        this.rotationSupplier = new BreakerSwerveRotationSupplier(() -> (BreakerMath.getPointAngleRelativeToOtherPoint(prevWp, waypoints.get(0))));
        driveController = config.getDriveController();
    }

    public BreakerSwerveWaypointFollower(BreakerSwerveWaypointFollowerConfig config, BreakerWaypointPath waypointPath, BreakerSwerveRotationSupplier rotationSupplier) {
        waypoints = new ArrayList<>();
        for (Translation2d wp: waypointPath.getWaypoints()) {
            waypoints.add(wp);
        }
        this.config = config;
        this.waypointPath = waypointPath;
        this.rotationSupplier = rotationSupplier;
        driveController = config.getDriveController();
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    BreakerLog.logBreakerLibEvent("A new BreakerSwerveWaypointFollower instance has started");
    prevWp = config.getOdometer().getOdometryPoseMeters().getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      driveController.calculate(null, null, 0, null);
      
  }

  public ArrayList<Translation2d> getWaypoints() {
    return waypoints;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BreakerLog.logBreakerLibEvent("A BreakerSwerveWaypointFollower instance has ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return auto.atPivotSetPoint();
  }


    
}
