package nodelet;

public interface NodeletUnloadResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "nodelet/NodeletUnloadResponse";
  static final java.lang.String _DEFINITION = "bool success";
  boolean getSuccess();
  void setSuccess(boolean value);
}
