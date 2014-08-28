//
// File:         stewart_platform_physics.c
// Date:         October 30, 2008
// Description:  Physics plugin file for the stewart_platform.wbt example
//               This plugin is used to attach the lower and upper piston ends
//               to the lower and upper cylinder platforms of the stewart platform
// Author:       Yvan Bourquin - www.cyberbotics.com
//

#include <ode/ode.h>
#include <plugins/physics.h>

#define NUM_PISTONS 6

// find ODE body by its DEF name in the .wbt file
dBodyID getBody(const char *def) {
  dBodyID body = dWebotsGetBodyFromDEF(def);
  if (! body) dWebotsConsolePrintf("Warning: did not find body with DEF name: %s", def);
  return body;
}

// function invoked by Webots
void webots_physics_init(dWorldID world, dSpaceID s, dJointGroupID j) {

  // general ODE objects that should be present in the .wbt model
  dBodyID lower_platform = getBody("STEWART_PLATFORM");
  dBodyID upper_platform = getBody("UPPER_PLATFORM");
  if (! lower_platform || ! upper_platform) {
    // if any object is missing it's safer to stop immediately otherwise
    // the NULL pointers will cause a segmentation fault and crash Webots.
    // If we stop now the example will not work but Webots will not crash.
    return;
  }

  int i;
  for (i = 0; i < NUM_PISTONS; i++) {
    // find lower piston body
    char name[64];
    sprintf(name, "LOWER_PISTON_%d", i);
    dBodyID lower_piston = getBody(name);
    if (! lower_piston) return;

    // create a universal joint (2 DOFs) to attach lower piston body to lower platform body
    // we need to use a universal joint to prevent the piston from passively rotating around its long axis
    dJointID universalJoint = dJointCreateUniversal(world, 0);
    dJointAttach(universalJoint, lower_piston, lower_platform);

    // transform attachement point from local to global coordinate system
    // warning: this hard-coded translation must match the one in generate_platform.c
    dVector3 lower_ball;
    dBodyGetRelPointPos(lower_piston, 0, -0.9, 0, lower_ball);

    // set attachement point (anchor)
    // and set the universal joint axes to be perpendicular to each other
    dJointSetUniversalAnchor(universalJoint, lower_ball[0], lower_ball[1], lower_ball[2]);
    dJointSetUniversalAxis1(universalJoint, 1, 0, 0);
    dJointSetUniversalAxis2(universalJoint, 0, 0, 1);

    // find upper piston's body
    dVector3 upper_ball;
    sprintf(name, "UPPER_PISTON_%d", i);
    dBodyID upper_piston = getBody(name);
    if (! upper_piston) return;

    // create a ball and socket joint (3 DOFs) to attach the upper piston body to the upper platform body
    // we don't need a universal joint here, because the piston's passive rotation is already prevented
    // by the universal joint at its lower end.
    dJointID upperBallJoint = dJointCreateBall(world, 0);
    dJointAttach(upperBallJoint, upper_piston, upper_platform);

    // warning: this hard-coded translation must match the one in generate_platform.c
    dBodyGetRelPointPos(upper_piston, 0, 0, -0.2, upper_ball);
    dJointSetBallAnchor(upperBallJoint, upper_ball[0], upper_ball[1], upper_ball[2]);
  }
}

void webots_physics_step() {
  // nothing to do ...
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  // collision not handled, otherwise should return 1
  return 0;
}

void webots_physics_cleanup() {
  // nothing to cleanup ...
}
