package GlobalNavigation;

import java.awt.geom.*;
import java.util.*;
import java.awt.Color;

public class MotionPlanner {

  public Grid grid;

  Point2D.Double currPoint;
  Point2D.Double addPoint;
  Point2D.Double checkPoint;


  MotionPlanner(Rectangle2D.Double worldBounds, double resolution, List<PolygonObstacle> obstacles) {
    grid = new Grid(worldBounds, resolution);

    for (PolygonObstacle obstacle: obstacles){
      grid.markObstacle(obstacle);
    }
  }

  List<Point2D.Double> computePath(Point2D.Double robotPoint, Point2D.Double goalPoint){
    List<Point2D.Double> path = new ArrayList<Point2D.Double>();
    List<Point2D.Double> shortPath = new ArrayList<Point2D.Double>();
    Grid.Cell goalCell = grid.getCell(goalPoint);
    Grid.Cell robotCell = grid.getCell(robotPoint);
    
    double maxDist = grid.computeShortestPaths(goalPoint);
   
    if (grid.getCell(robotPoint).toGoalNext==null){
    	System.out.println("if-2");
      return path;
    } else {
      if (grid.getCell(robotPoint).toGoalNext!=null){

        Grid.Cell currentCell = robotCell;
        while (!currentCell.getRect().contains(goalPoint)){
          path.add(currentCell.makeCenterPoint());
          currentCell = currentCell.toGoalNext;
        }
        path.add(currentCell.makeCenterPoint());
      } else {
        System.out.println("ERROR!: No path calculated.");
      }
    shortPath = shortenPath(path);
    return shortPath;
    //return path;
    }
  }
  
  List<Point2D.Double> shortenPath(List<Point2D.Double> path){
    List<Point2D.Double> shortPath = new ArrayList<Point2D.Double>();
    Point2D.Double goalPoint = path.get(path.size()-1);

    currPoint = path.get(0);
    addPoint = path.get(1);
    checkPoint = path.get(2);

    shortPath.add(currPoint);

    int i;

    
    System.out.println("Entering While");
    while (checkPoint.x!=goalPoint.x || checkPoint.y!=goalPoint.y){//worried
     System.out.println("Currently checking at x: "+ checkPoint.x +", y: " +checkPoint.y);
      if (isVisible(currPoint, checkPoint)){
        i = path.indexOf(checkPoint);
        addPoint = path.get(i);
       // System.out.println("Breaking 1");
        System.out.println(i);
        checkPoint = path.get(i+1);
       // System.out.println("Breaking 1.5");
      } else {
        shortPath.add(addPoint);
        currPoint = addPoint;
        i = path.indexOf(currPoint);
        System.out.println("Breaking 2");
        addPoint = path.get(i+1);
        System.out.println("Breaking 3");
        checkPoint = path.get(i+2); //worried outofbounds
        System.out.println("Breaking 4 "); //shouldnt get here.
      }
    }
    System.out.println("Exiting While");
    shortPath.add(addPoint);
    shortPath.add(checkPoint);
    return shortPath;
  }

  boolean isVisible(Point2D.Double currPoint, Point2D.Double checkPoint){
    double dx = checkPoint.x-currPoint.x;
    double dy = checkPoint.y-currPoint.y;
	System.out.println("dx: " + dx+", dy: "+dy);

    for (double j=0.0;j<100.0;j++){
      double newX = currPoint.x+j/100.0*dx;
      double newY = currPoint.y+j/100.0*dy;
      Point2D.Double newPoint = new Point2D.Double(newX, newY);
      Grid.Cell newCell = grid.getCell(newPoint);

      if (!newCell.free){
       // System.out.println("isVisible, NewCell: " +newCell.free);
        return false;
      }
else { //System.out.println("isVisible, NewCell: " + newCell.free);
}
    }
    return true;

  }

}


