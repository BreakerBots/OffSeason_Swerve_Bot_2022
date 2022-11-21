// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.waypoint.pathfinder;

import java.awt.geom.Rectangle2D;

/** Add your docs here. */
public class BreakerPathfinderNode {
    private int gridPosX, gridPosY;
    private boolean isBarrier;
    public BreakerPathfinderNode(int gridPosX, int gridPosY, boolean isBarrier) {
        this.gridPosX = gridPosX;
        this.gridPosY = gridPosY;
        this.isBarrier = isBarrier;
    }

    public boolean isBarrier() {
        return isBarrier;
    }

    public int getGridPosX() {
        return gridPosX;
    }

    public int getGridPosY() {
        return gridPosY;
    }

    public BreakerPathfinderNodeHeuristics getHeuristicWeight(BreakerPathfinderNode startNode, BreakerPathfinderNode endNode) {
        return new BreakerPathfinderNodeHeuristics(getManhattanCost(startNode), getManhattanCost(endNode));
    }

    private int getManhattanCost(BreakerPathfinderNode outher) {
        return calculateManhattanCost(gridPosX, gridPosY, outher.gridPosX, outher.gridPosY);
    }

    private static int calculateManhattanCost(int x0, int y0, int x1, int y1) {
        return Math.max(Math.abs(x1-x0), Math.abs(y1-y0));
    }

    public BreakerPathfinderNodeInstance getInstance(BreakerPathfinderNode pathStartNode, BreakerPathfinderNode pathEndNode) {
        return new BreakerPathfinderNodeInstance(this, pathStartNode, pathEndNode);
    }

    @Override
    public boolean equals(Object obj) {
        BreakerPathfinderNode outher = (BreakerPathfinderNode) obj;
        return gridPosX == outher.gridPosX && gridPosY == outher.gridPosY && isBarrier == outher.isBarrier;
    }

    public static class BreakerPathfinderNodeHeuristics {
        /** g = dist cur to start, f = dist cur to goal */
        private int gCost, fCost, hCost;
        public BreakerPathfinderNodeHeuristics(int gCost, int fCost) {
            this.gCost = gCost;
            this.fCost = fCost;
            hCost = fCost + hCost;
        }

        public int getF() {
            return fCost;
        }

        public int getG() {
            return gCost;
        }

        public int getH() {
            return hCost;
        }
    }

    public static class BreakerPathfinderNodeInstance {
        private BreakerPathfinderNode baseNode, pathStartNode, pathEndNode;
        private BreakerPathfinderNodeInstance parent;
        private BreakerPathfinderNodeHeuristics heuristics;
        private boolean isChecked, isOpen, isStart, isEnd, isParentSet;
        public BreakerPathfinderNodeInstance(BreakerPathfinderNode baseNode, BreakerPathfinderNode pathStartNode, BreakerPathfinderNode pathEndNode) {
            this.baseNode = baseNode;
            this.pathStartNode = pathStartNode;
            this.pathEndNode = pathEndNode;
            heuristics = baseNode.getHeuristicWeight(pathStartNode, pathEndNode);
            isStart = baseNode.equals(pathStartNode);
            isEnd = baseNode.equals(pathEndNode);
        }

        public void setParent(BreakerPathfinderNodeInstance parent) {
            this.parent = parent;
            isParentSet = true;
        }

        public BreakerPathfinderNodeInstance getParent() {
            return parent;
        }

        public boolean isParentSet() {
            return isParentSet;
        }

        public boolean isEnd() {
            return isEnd;
        }

        public boolean isStart() {
            return isStart;
        }

        public void setOpen(boolean isOpen) {
            this.isOpen = isOpen;
        }

        public void setChecked(boolean isChecked) {
            this.isChecked = isChecked;
        }

        public boolean isChecked() {
            return isChecked;
        }

        public boolean isOpen() {
            return isOpen;
        }

        public BreakerPathfinderNodeHeuristics getHeuristics() {
            return heuristics;
        }

        public BreakerPathfinderNode getBaseNode() {
            return baseNode;
        }

        public boolean isBarrier() {
            return baseNode.isBarrier;
        }
    
        public int getGridPosX() {
            return baseNode.gridPosX;
        }
    
        public int getGridPosY() {
            return baseNode.gridPosY;
        }

        @Override
        public boolean equals(Object obj) {
            BreakerPathfinderNodeInstance instance = (BreakerPathfinderNodeInstance) obj;
            return baseNode.equals(instance.getBaseNode());
        }
    }

}
