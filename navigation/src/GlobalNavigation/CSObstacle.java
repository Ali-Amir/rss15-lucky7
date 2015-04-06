package GlobalNavigation;

import java.awt.geom.*;
import java.util.ArrayList;
import java.util.List;

/**
 * <p>Class that describes obstacles in C-Space.</p>
 **/
public class CSObstacle {
  public final static int ANGLE_DIVISIONS = 100;
  protected List<PolygonObstacle> polyRot;

  CSObstacle() {
    polyRot = new ArrayList<PolygonObstacle>();
  }

  CSObstacle(PolygonObstacle obst) {
    polyRot = new ArrayList<PolygonObstacle>();

    for (int i = 0; i < ANGLE_DIVISIONS; ++i) {
      double theta = 2*Math.PI/ANGLE_DIVISIONS*i;

      polyRot.add(CSTools.getCSTransform(obst, theta));
    }
  }

  PolygonObstacle get(int index) {
    return polyRot.get(index);
  }
  
  public boolean intersects(Rectangle2D.Double rect, int angInd) {
    for (int ind = angInd-1; ind <= angInd+1; ++ind) {
      if (ind >= 0 && ind < ANGLE_DIVISIONS &&
          polyRot.get(ind).intersects(rect)) {
        return true;
      }
    }
    return false;
  }

  public boolean contains(CSCoord s) {
    int ind = s.thetaInd();
    return polyRot.get(ind).intersects(
        new Rectangle2D.Double(s.x(), s.y(), 1e-3, 1e-3));
  }

  public boolean intersects(CSCoord s, CSCoord e) {
    for (int step = 0; step <= 100; ++step) {
      double x = s.x() + (e.x()-s.x())*step/100.0;
      double y = s.y() + (e.y()-s.y())*step/100.0;
      int thetaInd =
          (int) Math.round(s.thetaInd() + (e.thetaInd()-s.thetaInd())*step/100.0);

      if (contains(new CSCoord(x, y, thetaInd))) {
        return true;
      }
    }
    return false;
  }
}
