package baxter_adapt;

public interface Adaptation extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_adapt/Adaptation";
  static final java.lang.String _DEFINITION = "#request imitation trajectory \ngeometry_msgs/Point y_start\ngeometry_msgs/Point y_end\ngeometry_msgs/Point[] obstacles\n---\n#response constants\nbool response\nstring filename\n";
}
