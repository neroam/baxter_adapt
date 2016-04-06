package baxter_adapt;

public interface ImitationRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_adapt/ImitationRequest";
  static final java.lang.String _DEFINITION = "#request imitation trajectory \ngeometry_msgs/Point y_start\ngeometry_msgs/Point y_end\n";
  geometry_msgs.Point getYStart();
  void setYStart(geometry_msgs.Point value);
  geometry_msgs.Point getYEnd();
  void setYEnd(geometry_msgs.Point value);
}
