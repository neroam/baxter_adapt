package baxter_adapt;

public interface LearningRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_adapt/LearningRequest";
  static final java.lang.String _DEFINITION = "#request weights learning \nstring filename\n";
  java.lang.String getFilename();
  void setFilename(java.lang.String value);
}
