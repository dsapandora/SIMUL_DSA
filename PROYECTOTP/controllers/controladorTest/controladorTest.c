#include <webots/robot.h>

// Added a new include file
#include <webots/differential_wheels.h>

#define TIME_STEP 64

int main(int argc, char **argv)
{
  wb_robot_init();
  
  // set up the speeds
  wb_differential_wheels_set_speed(100, 100);
  
  do {
  } while (wb_robot_step(TIME_STEP) != -1);
  
  wb_robot_cleanup();
  
  return 0;
}