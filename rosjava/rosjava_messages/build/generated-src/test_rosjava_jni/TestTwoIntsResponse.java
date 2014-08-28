package test_rosjava_jni;

public interface TestTwoIntsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rosjava_jni/TestTwoIntsResponse";
  static final java.lang.String _DEFINITION = "int32 sum";
  int getSum();
  void setSum(int value);
}
