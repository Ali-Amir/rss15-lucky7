/*
 * Copyright (C) 2014 Ali-Amir Aldan.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava.challenge.navigation;

import java.awt.geom.*;
import java.util.*;

/**
 * <p>Grid for path planning.</p>
 *
 * @author vona
 **/
public class CSGrid {

  public class Cell {

    /**
     * <p>This cell's row.</p>
     **/ 
    int r;

    /**
     * <p>This cell's column.</p>
     **/
    int c;

    /**
     * <p>This cell's angle position.</p>
     **/
    int angleId;

    /**
     * <p>The bounds of this cell in world coords.</p>
     **/
    Rectangle2D.Double rect = new Rectangle2D.Double();

    /**
     * <p>Distance to goal along the shortest path (m), iff {@link
     * Grid#computeShortestPaths} has been run.</p>.
     **/
    double minDistanceToGoal = Double.POSITIVE_INFINITY;

    /**
     * <p>Whether this cell is in free space.</p>
     **/
    boolean free = true;

    public double x;
    public double y;


    /**
     * <p>The next cell along the shortest path to the goal, if any, iff {@link
     * Grid#computeShortestPaths} has been run.</p>
     *
     * <p>Null if this cell contains the goal.</p>
     **/
    Cell toGoalNext = null;

    /**
     * <p>Create a new cell.</p>
     **/
    Cell(int r, int c, int angleId, double x, double y, double width, double height) {
      this.r = r;
      this.c = c;
      this.angleId = angleId;
      this.x = x;
      this.y = y;
      rect.x = x;
      rect.y = y;
      rect.width = width;
      rect.height = height;
    }
    
    public Rectangle2D.Double getRect(){
    	return rect;
    }

    /**
     * <p>Get the x coordinate of the center of this cell.</p>
     *
     * @return the x coordinate of the center of this cell
     **/
    public double getCenterX() {
      return rect.x + rect.width/2.0;
    }

    /**
     * <p>Get the y coordinate of the center of this cell.</p>
     *
     * @return the y coordinate of the center of this cell
     **/
    public double getCenterY() {
      return rect.y + rect.height/2.0;
    }

    /**
     * <p>Return the center point of this cell.</p>
     *
     * @return the center point of this cell as a new point
     **/
    public Point2D.Double makeCenterPoint() {
      return new Point2D.Double(getCenterX(), getCenterY());
    }

    /**
     * <p>Check whether this cell has a path to the goal.</p>
     *
     * <p>Returns true even if this cell contains the goal (i.e. {@link
     * #toGoalNext} is null).</p>
     *
     * @return true iff this cell has a path to the goal
     **/
    public boolean hasPathToGoal() {
      return free && !Double.isInfinite(minDistanceToGoal);
    }

    /**
     * <p>Return a human-readable string representation of this cell.</p>
     **/
    public String toString() {
      return "(" + r + ", " + c + ", " + angleId + ") x=" +
        rect.x + ", y=" + rect.y +
        ", width=" + rect.width + ", height=" + rect.height +
        ", dist=" + minDistanceToGoal;
    }
  }

  /**
   * <p>World boundary.</p>
   **/
  protected Rectangle2D.Double worldBounds;

  /**
   * <p>Grid resolution.</p>
   **/
  protected double resolution;

  /**
   * <p> Number of divisions in the angle space.</p>
   **/
  protected int angleDiv;

  /**
   * <p>Number of rows in the grid.</p>
   **/
  protected int rows;

  /**
   * <p>Number of cols in the grid.</p>
   **/
  protected int cols;

  /**
   * <p>The grid cells.</p>
   **/
  protected Cell[][][] cell;

  /**
   * <p>Create a new grid.</p>
   *
   * @param worldBounds the world boundary
   * @param resolution the grid resolution
   **/
  public CSGrid(Rectangle2D.Double worldBounds, int angleDivisions, double resolution) {

    this.worldBounds = worldBounds;
    this.resolution = resolution;
    this.angleDiv = angleDivisions;
    
    // Double the size of the grid to enable rotations.
    rows = (int) Math.ceil(worldBounds.height*2.0/resolution);
    cols = (int) Math.ceil(worldBounds.width*2.0/resolution);

    cell = new Cell[angleDiv][rows][cols];

    double worldCenterX = worldBounds.x + worldBounds.width/2.0;
    double worldCenterY = worldBounds.y + worldBounds.height/2.0;
    for (int angleId = 0; angleId < angleDiv; ++angleId) {
      double ux = Math.cos(2.0*Math.PI*angleId/angleDiv);
      double uy = Math.sin(2.0*Math.PI*angleId/angleDiv);
      double vx = Math.cos(2.0*Math.PI*angleId/angleDiv+Math.PI/2.0);
      double vy = Math.sin(2.0*Math.PI*angleId/angleDiv+Math.PI/2.0);
      for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
          int dr = (rows >> 1) - r, dc = (cols >> 1) - c;
          cell[angleId][r][c] =
            new Cell(r, c, angleId,
                     worldCenterX + (dc*ux + dr*vx)*resolution, worldCenterY + (dc*uy + dr*vy)*resolution,
                     resolution, resolution);
        }
      }
    }
  }

  /**
   * <p>Mark all the {@link Grid.Cell} that {@link PolygonObstacle#intersects}
   * <code>obstacle</code> not {@link Grid.Cell#free}.</p>
   *
   * @param obstacle the obstacle
   *
   * @return the number of cells marked
   **/
  public int markObstacle(CSObstacle obstacle) {

    int n = 0;

    for (int angleId = 0; angleId < angleDiv; ++angleId) {
      for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
          if (obstacle.intersects(cell[angleId][r][c].rect, angleId)) {
            cell[angleId][r][c].free = false;
            ++n;
          }
        }
      }
    }

    return n;
  }

  /**
   * <p>Compute and cache the shortest (Manhattan) paths (BFS) from each cell
   * in the grid to the cell containing <code>goalPoint</code>.</p>
   *
   * <p>Only {@link Cell#free} cells are traversed.</p>
   *
   * @param goalPoint the goal point
   *
   * @return the maximum path distance from any cell which can reach the goal
   * to the goal, or Double.POSITIVE_INFINITY if <code>goalPoint</code> is out
   * of bounds
   **/
  public double computeShortestPaths(Point2D.Double goalPoint) {

    List<Cell> goalCells = getCell(goalPoint);

    if (goalCells == null || goalCells.size() == 0)
      return Double.POSITIVE_INFINITY;

    LinkedList<Cell> queue = new LinkedList<Cell>();
    LinkedList<Cell> neighbors = new LinkedList<Cell>();

    for (Cell goal : goalCells) {
      goal.minDistanceToGoal = 0.0;
      queue.add(goal);
    }

    return runBfs(queue, neighbors);
  }

  public double computeShortestPaths(CSCoord goalPoint) {

    Cell goalCell = getCell(goalPoint);

    if (goalCell == null)
      return Double.POSITIVE_INFINITY;
   
    goalCell.minDistanceToGoal = 0.0;

    LinkedList<Cell> queue = new LinkedList<Cell>();
    LinkedList<Cell> neighbors = new LinkedList<Cell>();

    queue.add(goalCell);
    return runBfs(queue, neighbors);
  }

  public double runBfs(LinkedList<Cell> queue, LinkedList<Cell> neighbors) {
    double maxDist = 0.0;

    while (!queue.isEmpty()) {
      Cell cell = queue.removeFirst();
      neighbors.clear();
      for (Cell neighbor : collectNeighbors(cell, neighbors)) {
        if (neighbor.free && Double.isInfinite(neighbor.minDistanceToGoal)) {
          neighbor.minDistanceToGoal = cell.minDistanceToGoal + resolution;
          neighbor.toGoalNext = cell;
          queue.add(neighbor);
          if (neighbor.minDistanceToGoal > maxDist)
            maxDist = neighbor.minDistanceToGoal;
        }
      }
    }

    return maxDist;
  }


  /**
   * <p>Collect all the (Manhattan) neighbors of <code>cell</code>.</p>
   *
   * @param c the cell
   * @param neighbors the neighbors are collected here
   * @return a ref to <code>neighbors</code>
   **/ 
  protected List<Cell> collectNeighbors(Cell c, List<Cell> neighbors) {

    //below
    if (c.r > 0)
      neighbors.add(cell[c.angleId][c.r-1][c.c]);

    //above
    if (c.r < rows-1)
      neighbors.add(cell[c.angleId][c.r+1][c.c]);

    //left
    if (c.c > 0)
      neighbors.add(cell[c.angleId][c.r][c.c-1]);

    //right

    if (c.c < cols-1)
      neighbors.add(cell[c.angleId][c.r][c.c+1]);

    for (int i = 0; i < angleDiv; ++i) {
      if (i != c.angleId) {
        Cell verticalNeighbor = getCell(new CSCoord(c.x, c.y, i));
        if (verticalNeighbor != null) {
          neighbors.add(verticalNeighbor);
        }
      }
    }

    return neighbors;
  }

  /**
   * <p>Get the cell containing <code>point</code>, or null if out of
   * bounds.</p>
   *
   * @param point the query point
   *
   * @return the cell containing <code>point</code>, or null if out of bounds
   **/
  public Cell getCell(CSCoord point) {

    if (!worldBounds.contains(point.coord())) {
      return null;
    }

    double worldCenterX = worldBounds.x + worldBounds.width/2.0;
    double worldCenterY = worldBounds.y + worldBounds.height/2.0;
    int angleId = point.thetaInd;

    double ux = Math.cos(2.0*Math.PI*angleId/angleDiv);
    double uy = Math.sin(2.0*Math.PI*angleId/angleDiv);
    double vx = Math.cos(2.0*Math.PI*angleId/angleDiv+Math.PI/2.0);
    double vy = Math.sin(2.0*Math.PI*angleId/angleDiv+Math.PI/2.0);

    // Solve system of linear equations:
    // dc*ux + dr*vx = varX
    // dc*uy + dr*vy = varY
    double varX = (point.x()-worldCenterX)/resolution;
    double varY = (point.y()-worldCenterY)/resolution;
    double detA = (ux*vy - uy*vx);

    int dc = (int)((varX*vy - varY*vx)/detA);
    int dr = (int)((ux*varY - uy*varX)/detA);

    int actualC = Math.max(0, Math.min(cols-1, (cols>>1) - dc));
    int actualR = Math.max(0, Math.min(rows-1, (rows>>1) - dr));

    return cell[angleId][actualR][actualC];
  }

  public LinkedList<Cell> getCell(Point2D.Double point) {

    if (!worldBounds.contains(point)) {
      return null;
    }

    double worldCenterX = worldBounds.x + worldBounds.width/2.0;
    double worldCenterY = worldBounds.y + worldBounds.height/2.0;

    LinkedList<Cell> result = new LinkedList<Cell>();

    for (int angleId = 0; angleId < angleDiv; ++angleId) {
      double ux = Math.cos(2.0*Math.PI*angleId/angleDiv);
      double uy = Math.sin(2.0*Math.PI*angleId/angleDiv);
      double vx = Math.cos(2.0*Math.PI*angleId/angleDiv+Math.PI/2.0);
      double vy = Math.sin(2.0*Math.PI*angleId/angleDiv+Math.PI/2.0);

      // Solve system of linear equations:
      // dc*ux + dr*vx = varX
      // dc*uy + dr*vy = varY
      double varX = (point.getX()-worldCenterX)/resolution;
      double varY = (point.getY()-worldCenterY)/resolution;
      double detA = (ux*vy - uy*vx);

      int dc = (int)((varX*vy - varY*vx)/detA);
      int dr = (int)((ux*varY - uy*varX)/detA);

      int actualC = Math.max(0, Math.min(cols-1, (cols>>1) - dc));
      int actualR = Math.max(0, Math.min(rows-1, (rows>>1) - dr));

      result.add(cell[angleId][actualR][actualC]);
    }

    return result;
  }
}
