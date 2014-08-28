package ROSARIA;

public interface BumperState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ROSARIA/BumperState";
  static final java.lang.String _DEFINITION = "Header header\nbool[] front_bumpers\nbool[] rear_bumpers\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean[] getFrontBumpers();
  void setFrontBumpers(boolean[] value);
  boolean[] getRearBumpers();
  void setRearBumpers(boolean[] value);
}
