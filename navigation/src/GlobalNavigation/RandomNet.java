package GlobalNavigation;

import java.awt.geom.*;
import java.util.*;
import java.lang.Integer;

/**
 * <p>Random tree for path planning.</p>
 *
 * @author aliamir 
 **/
public class RandomNet {

  /**
   * <p>World information.</p>
   **/
  protected Rectangle2D.Double worldBounds;
  protected List<CSObstacle> obstacles;
  /**
   * <p>Random tree elements.</p>
   **/
  protected int numPoints;
  protected List<CSCoord> points;
  protected List<List<Integer>> edges;
  /**
   * <p>Dfs elements.</p>
   **/
  protected int pathGoal;
  protected List<Integer> visited;
  protected List<Integer> next;

  /**
   * <p>Create a random tree.</p>
   *
   * @param worldBounds the world boundary
   * @param obstacles obstacles in C-Space
   * @param numPoints number of points to be generated
   **/
  public RandomNet(Rectangle2D.Double worldBounds,
                   List<CSObstacle> obstacles, int numPoints) {

    this.obstacles = obstacles;
    this.numPoints = numPoints;
    this.worldBounds = worldBounds;

    buildTree();
  }

  public List<CSCoord> getPoints() {
    return this.points;
  }

  public boolean getContainedInObstacle(CSCoord point) {
    for (CSObstacle obstacle : this.obstacles) {
      if (obstacle.contains(point)) {
        return true;
      }
    }
    return false;
  }

  /***
   * <p>Build the random tree by adding one random point after another.</p>
   **/
  public void buildTree() {
    this.points = new ArrayList<CSCoord>();
    this.points.add(new CSCoord(0.0, 0.0, 0));
    this.next = new ArrayList<Integer>(this.numPoints);
    this.visited = new ArrayList<Integer>(this.numPoints);

    Random randomGenerator = new Random(3141592653l);
    while (this.points.size() < this.numPoints) {
      int ind = randomGenerator.nextInt(this.points.size());

      double x = randomGenerator.nextDouble()*this.worldBounds.width;
      double y = randomGenerator.nextDouble()*this.worldBounds.height;
      int thetaInd = this.points.get(ind).thetaInd();
      if (randomGenerator.nextInt(10) == 0) {
        x = this.points.get(ind).x();
        y = this.points.get(ind).y();
        int delta = CSObstacle.ANGLE_DIVISIONS/12;
        thetaInd = Math.max(0, randomGenerator.nextInt(2*delta)-delta+
                               this.points.get(ind).thetaInd());
      }
      CSCoord newPoint = new CSCoord(x, y, thetaInd);
      // Check if the new point lies within one of the obstacles.
      if (getContainedInObstacle(newPoint)) {
        continue;
      }
      // Find the closest point from current list of points.
      int closest = getClosestVertex(newPoint);
      // If the new point is to far away from current tree, skip it.
      if (this.points.get(closest).distanceTo(newPoint) > 2.0) {
        continue;
      }
      // Check for intersections with obstacles if we have a connection.
      boolean noIntersections = true;
      if (closest >= 0) {
        for (CSObstacle obstacle : this.obstacles) {
          if (obstacle.intersects(newPoint, this.points.get(closest))) {
            noIntersections = false;
            break;
          }
        }
      }
      if (noIntersections) {
        int verId = this.points.size();
        this.points.add(newPoint);
        this.edges.add(new ArrayList<Integer>());
        this.edges.get(closest).add(verId);
        this.edges.get(verId).add(closest);
      }
    }
  }

  /***
   * <p>Searches paths to the goal vertex using DFS.</p>
   **/
  protected void dfs(int u, int prev) {
    this.next.set(u, new Integer(prev));
    for (int v : this.edges.get(u)) {
      if (v != prev) {
        dfs(v, u);
      }
    }
  }

  protected int getClosestVertex(CSCoord v) {
    int closest = -1;
    for (int i = 0; i < this.points.size(); ++i) {
      if (closest == -1 || this.points.get(i).distanceTo(v) <
                              this.points.get(closest).distanceTo(v)) {
        closest = i;
      }
    }
    return closest;
  }

  /***
   * <p>Find a path from all vertices to a goalPoint by mapping latter to
   * the closest random tree point and doing a DFS.</p>
   **/
  public void computeShortestPathsToGoal(CSCoord goalPoint) {

    int goalVer = getClosestVertex(goalPoint);
    this.pathGoal = goalVer;

    dfs(goalVer, -1);
  }

  public boolean getNextPointOrExit(CSCoord curPoint,
                                    CSCoord nextPoint) {

    int curVer = getClosestVertex(curPoint);

    if (curVer == this.pathGoal) {
      return false;
    }

    nextPoint.reset(this.points.get(this.next.get(curVer)));
    
    return true;
  }

}
