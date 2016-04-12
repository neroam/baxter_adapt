package baxter_adapt;

public interface Learning extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_adapt/Learning";
  static final java.lang.String _DEFINITION = "#request weights learning \nstring filename\n---\n#response constants\nbool response\nstring filename\n";
}
