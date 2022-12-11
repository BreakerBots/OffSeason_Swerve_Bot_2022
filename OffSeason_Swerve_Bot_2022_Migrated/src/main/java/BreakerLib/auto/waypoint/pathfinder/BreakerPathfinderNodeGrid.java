// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BreakerLib.auto.waypoint.pathfinder;

import java.awt.geom.Rectangle2D;
import java.util.Arrays;

import BreakerLib.auto.waypoint.pathfinder.BreakerPathfinderNode.BreakerPathfinderNodeInstance;
import BreakerLib.position.geometry.BreakerFieldGeometry2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Grid of nodes for BreakerPathfinder. */
public class BreakerPathfinderNodeGrid {

    private BreakerPathfinderNode[][] nodeGrid;
    private Rectangle2D.Double[][] nodeBounds;
    private double nodeSideLengthMeters;

    /** */
    public BreakerPathfinderNodeGrid(int nodesX, int nodesY, double nodeSideLengthMeters,
            BreakerFieldGeometry2d fieldGeometry) {
        nodeGrid = new BreakerPathfinderNode[nodesX][nodesY];
        nodeBounds = new Rectangle2D.Double[nodesX][nodesY];
        this.nodeSideLengthMeters = nodeSideLengthMeters;
        for (int i = 0; i < nodesX; i++) {
            for (int j = 0; j < nodesY; j++) {
                nodeBounds[i][j] = new Rectangle2D.Double((double) i, (double) j + 1, nodeSideLengthMeters,
                        nodeSideLengthMeters);
                nodeGrid[i][j] = new BreakerPathfinderNode(i, j, fieldGeometry.intersects(nodeBounds[i][j]));
            }
        }
    }

    public BreakerPathfinderNodeGrid(int nodesX, int nodesY, double nodeSideLengthMeters,
            BreakerPathfinderNode... preDefinedNodes) {
        nodeGrid = new BreakerPathfinderNode[nodesY][nodesX];
        nodeBounds = new Rectangle2D.Double[nodesX][nodesY];
        this.nodeSideLengthMeters = nodeSideLengthMeters;
        for (int i = 0; i < nodesX; i++) {
            for (int j = 0; j < nodesY; j++) {
                for (BreakerPathfinderNode node : preDefinedNodes) {
                    if (node.getGridPosX() == i && node.getGridPosY() == j) {
                        nodeGrid[i][j] = node;
                    } else {
                        nodeGrid[i][j] = new BreakerPathfinderNode(i, j, false);
                    }
                }
                nodeBounds[i][j] = new Rectangle2D.Double((double) i, (double) j + 1, nodeSideLengthMeters,
                        nodeSideLengthMeters);
            }
        }
    }

    public double getNodeSideLengthMeters() {
        return nodeSideLengthMeters;
    }

    public BreakerPathfinderNodeGridInstance getInstance(BreakerPathfinderNode startNode,
            BreakerPathfinderNode endNode) {
        return new BreakerPathfinderNodeGridInstance(nodeSideLengthMeters, startNode, endNode, nodeGrid, nodeBounds);
    }

    public BreakerPathfinderNode getNodeFromPosition(Translation2d position) throws NodeNotFoundWithinGridException {
        for (int i = 0; i < nodeBounds.length; i++) {
            for (int j = 0; j < nodeBounds[i].length; j++) {
                if (nodeBounds[i][j].contains(position.getX(), position.getY())) {
                    return nodeGrid[i][j];
                }
            }
        }
        throw new NodeNotFoundWithinGridException(position);
    }

    public static class BreakerPathfinderNodeGridInstance {
        private BreakerPathfinderNodeInstance[][] instanceNodeGrid;
        private BreakerPathfinderNodeInstance startNodeInstance;
        private BreakerPathfinderNodeInstance endNodeInstance;
        private double nodeSideLengthMeters;
        private Rectangle2D.Double[][] nodeBounds;

        public BreakerPathfinderNodeGridInstance(double nodeSideLengthMeters, BreakerPathfinderNode startNode,
                BreakerPathfinderNode endNode, BreakerPathfinderNode[][] nodeGrid, Rectangle2D.Double[][] nodeBounds) {
            this.nodeSideLengthMeters = nodeSideLengthMeters;
            this.nodeBounds = Arrays.copyOf(nodeBounds, nodeBounds.length);
            startNodeInstance = startNode.getInstance(startNode, endNode);
            endNodeInstance = endNode.getInstance(startNode, endNode);
            instanceNodeGrid = new BreakerPathfinderNodeInstance[nodeGrid.length][];
            for (int i = 0; i < nodeGrid.length; i++) {
                instanceNodeGrid[i] = new BreakerPathfinderNodeInstance[nodeGrid[i].length];
                for (int j = 0; j < nodeGrid[i].length; j++) {
                    instanceNodeGrid[i][j] = nodeGrid[i][j].getInstance(startNode, endNode);
                }
            }
        }

        public BreakerPathfinderNodeInstance getEndNodeInstance() {
            return endNodeInstance;
        }

        public BreakerPathfinderNodeInstance getStartNodeInstance() {
            return startNodeInstance;
        }

        public BreakerPathfinderNodeInstance[][] getInstanceNodeGridArray() {
            return Arrays.copyOf(instanceNodeGrid, instanceNodeGrid.length);
        }

        public double getNodeSideLengthMeters() {
            return nodeSideLengthMeters;
        }

        public BreakerPathfinderNodeInstance getNodeInstanceFromPosition(Translation2d position)
                throws NodeNotFoundWithinGridException {
            for (int i = 0; i < nodeBounds.length; i++) {
                for (int j = 0; j < nodeBounds[i].length; j++) {
                    if (nodeBounds[i][j].contains(position.getX(), position.getY())) {
                        return instanceNodeGrid[i][j];
                    }
                }
            }
            throw new NodeNotFoundWithinGridException(position);
        }
    }

    public static class NodeNotFoundWithinGridException extends Exception {
        public NodeNotFoundWithinGridException(Translation2d point) {
            super(String.format(
                    "The BreakerPathfinderNode or BreakerPathfinderNodeInstance containing the point %s was not found within the given grid ",
                    point.toString()));
        }
    }
}
