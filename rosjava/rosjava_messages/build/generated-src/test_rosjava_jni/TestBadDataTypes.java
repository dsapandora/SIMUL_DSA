package test_rosjava_jni;

public interface TestBadDataTypes extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rosjava_jni/TestBadDataTypes";
  static final java.lang.String _DEFINITION = "# Unfortunately, can\'t test these fully because roscpp message generation\n# is broken. \n\nstd_msgs/Byte[2] Byte_f\nstd_msgs/ByteMultiArray[1] ByteMultiArray_f\n";
  java.util.List<std_msgs.Byte> getByteF();
  void setByteF(java.util.List<std_msgs.Byte> value);
  java.util.List<std_msgs.ByteMultiArray> getByteMultiArrayF();
  void setByteMultiArrayF(java.util.List<std_msgs.ByteMultiArray> value);
}
