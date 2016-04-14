package baxter_adapt;

public interface AdaptationRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_adapt/AdaptationRequest";
  static final java.lang.String _DEFINITION = "#request trajectory \nfloat64[] start_joints\nfloat64[] end_joints\ngeometry_msgs/Point[] obstacles\nbool enable_adapt \n";
  double[] getStartJoints();
  void setStartJoints(double[] value);
  double[] getEndJoints();
  void setEndJoints(double[] value);
  java.util.List<geometry_msgs.Point> getObstacles();
  void setObstacles(java.util.List<geometry_msgs.Point> value);
  boolean getEnableAdapt();
  void setEnableAdapt(boolean value);
}
