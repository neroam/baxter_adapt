package baxter_adapt;

public interface Adaptation extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_adapt/Adaptation";
  static final java.lang.String _DEFINITION = "#request trajectory \nfloat64[] start_joints\nfloat64[] end_joints\ngeometry_msgs/Point[] obstacles\nbool enable_adapt \n---\n#response constants\nbool response\nstring filename\n";
}
