package test_rosjava_jni;

public interface AddTwoIntsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rosjava_jni/AddTwoIntsResponse";
  static final java.lang.String _DEFINITION = "int64 sum";
  long getSum();
  void setSum(long value);
}
