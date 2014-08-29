package std_msgs;

public interface UInt16MultiArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "std_msgs/UInt16MultiArray";
  static final java.lang.String _DEFINITION = "# Please look at the MultiArrayLayout message definition for\n# documentation on all multiarrays.\n\nMultiArrayLayout  layout        # specification of data layout\nuint16[]            data        # array of data\n\n";
  std_msgs.MultiArrayLayout getLayout();
  void setLayout(std_msgs.MultiArrayLayout value);
  short[] getData();
  void setData(short[] value);
}
