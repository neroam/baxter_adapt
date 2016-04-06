package baxter_adapt;

public interface AdaptationRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_adapt/AdaptationRequest";
  static final java.lang.String _DEFINITION = "#request imitation trajectory \ngeometry_msgs/Point y_start\ngeometry_msgs/Point y_end\ngeometry_msgs/Point[] obstacles\n";
  geometry_msgs.Point getYStart();
  void setYStart(geometry_msgs.Point value);
  geometry_msgs.Point getYEnd();
  void setYEnd(geometry_msgs.Point value);
  java.util.List<geometry_msgs.Point> getObstacles();
  void setObstacles(java.util.List<geometry_msgs.Point> value);
}
