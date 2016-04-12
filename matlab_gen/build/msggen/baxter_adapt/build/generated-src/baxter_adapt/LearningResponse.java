package baxter_adapt;

public interface LearningResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_adapt/LearningResponse";
  static final java.lang.String _DEFINITION = "#response constants\nbool response\nstring filename";
  boolean getResponse();
  void setResponse(boolean value);
  java.lang.String getFilename();
  void setFilename(java.lang.String value);
}
