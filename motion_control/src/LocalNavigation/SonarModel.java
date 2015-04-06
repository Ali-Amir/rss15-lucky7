package LocalNavigation;

public class SonarModel {

  protected static int WALL_CLEARED_THRESH = 5;

  protected Robot robot;
  protected WallModel wallModel;
  protected String sonarName;

  public boolean isReadingWall = false;
  public boolean haveSeenWall = false;
  public int clearedWallCount = 0;

  public SonarModel(Robot robotPtr, WallModel wallModelPtr, String name) {
    robot = robotPtr;
    wallModel = wallModelPtr;
    sonarName = name;
  }

  public void updateSonar(SonarReading reading) {
    double p[] = LocalNavigation.getCoordsOffset(
        robot,
        LocalNavigation.SONAR_REL_X[reading.sonar],
        LocalNavigation.SONAR_REL_Y[reading.sonar]+reading.range);
    isReadingWall = wallModel.isPartOfWall(reading.range, p[0], p[1]);
    if (haveSeenWall != isReadingWall && !haveSeenWall) {
      System.out.println("Sonar " + sonarName + " has just seen the wall.");
    }
    haveSeenWall |= isReadingWall;
    if (clearedWallCount < WALL_CLEARED_THRESH && haveSeenWall) {
      if (!isReadingWall) {
        ++clearedWallCount;
        System.out.println("Cleared wall counter on sonar " + sonarName + ": " +
                           clearedWallCount);
      } else {
        clearedWallCount = 0;
      }
    }
  }

  public void reset() {
    isReadingWall = false;
    haveSeenWall = false;
    clearedWallCount = 0;
    System.out.println("Reset sonar " + sonarName);
  }

  public boolean hasClearedWall() {
    return clearedWallCount >= WALL_CLEARED_THRESH;
  }
}
