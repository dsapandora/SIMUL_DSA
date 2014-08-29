package gazebo_msgs;

public interface ODEPhysics extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/ODEPhysics";
  static final java.lang.String _DEFINITION = "bool auto_disable_bodies           # enable auto disabling of bodies, default false\nuint32 sor_pgs_precon_iters        # preconditioning inner iterations when uisng projected Gauss Seidel\nuint32 sor_pgs_iters               # inner iterations when uisng projected Gauss Seidel\nfloat64 sor_pgs_w                  # relaxation parameter when using projected Gauss Seidel, 1 = no relaxation\nfloat64 sor_pgs_rms_error_tol      # rms error tolerance before stopping inner iterations\nfloat64 contact_surface_layer      # contact \"dead-band\" width\nfloat64 contact_max_correcting_vel # contact maximum correction velocity\nfloat64 cfm                        # global constraint force mixing\nfloat64 erp                        # global error reduction parameter\nuint32 max_contacts                # maximum contact joints between two geoms\n";
  boolean getAutoDisableBodies();
  void setAutoDisableBodies(boolean value);
  int getSorPgsPreconIters();
  void setSorPgsPreconIters(int value);
  int getSorPgsIters();
  void setSorPgsIters(int value);
  double getSorPgsW();
  void setSorPgsW(double value);
  double getSorPgsRmsErrorTol();
  void setSorPgsRmsErrorTol(double value);
  double getContactSurfaceLayer();
  void setContactSurfaceLayer(double value);
  double getContactMaxCorrectingVel();
  void setContactMaxCorrectingVel(double value);
  double getCfm();
  void setCfm(double value);
  double getErp();
  void setErp(double value);
  int getMaxContacts();
  void setMaxContacts(int value);
}
