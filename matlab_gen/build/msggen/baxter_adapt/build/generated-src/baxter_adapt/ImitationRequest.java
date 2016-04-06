package baxter_adapt;

public interface ImitationRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_adapt/ImitationRequest";
  static final java.lang.String _DEFINITION = "#request imitation trajectory \nfloat64[] y_start\nfloat64[] y_end\n";
  double[] getYStart();
  void setYStart(double[] value);
  double[] getYEnd();
  void setYEnd(double[] value);
}
