package test_rosjava_jni;

public interface TestTwoIntsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_rosjava_jni/TestTwoIntsRequest";
  static final java.lang.String _DEFINITION = "int32 a\nint32 b\n";
  int getA();
  void setA(int value);
  int getB();
  void setB(int value);
}
