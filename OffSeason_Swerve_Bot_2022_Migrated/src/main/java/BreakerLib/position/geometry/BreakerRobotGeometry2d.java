// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.position.geometry;

import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;
import java.awt.geom.AffineTransform;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class BreakerRobotGeometry2d {
    private Area robotArea;

    public BreakerRobotGeometry2d(Area robotArea) {
        this.robotArea = robotArea;
    }

    public Rectangle2D getTranslatedHitbox(Pose2d robotPose) {
        Area a = new Area(robotArea);
        a.transform(AffineTransform.getRotateInstance(robotPose.getRotation().getRadians()));
        a.transform(AffineTransform.getTranslateInstance(robotPose.getX(), robotPose.getY()));
        return a.getBounds2D();
    }

    public Area getArea() {
        return robotArea;
    }




}
