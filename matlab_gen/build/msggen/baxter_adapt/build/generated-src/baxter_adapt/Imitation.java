package baxter_adapt;

public interface Imitation extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_adapt/Imitation";
  static final java.lang.String _DEFINITION = "#request imitation trajectory \nfloat64[] y_start\nfloat64[] y_end\n---\n#response constants\nbool response\nstring filename\n";
}
