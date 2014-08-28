/****************************************************************************

salamander_physics.c -- Hydrodynamics model for salamander.wbt simulation in Webots.
  
Copyright (C) 2007 Biorobotics Laboratory, EPFL, Lausanne
Authors:  Jerome Braure (Original) and Yvan Bourquin (Revised)
Web:      http://biorob.epfl.ch/

The authors of any publication arising from research using this software are
kindly requested to add the following reference:

  A. Ijspeert, A. Crespi, D. Ryczko, and J.M. Cabelguen.
  From swimming to walking with a salamander robot driven by a spinal cord model.
  Science, 315(5817):1416-1420, 2007.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

******************************************************************************/

#include <ode/ode.h>
#include <plugins/physics.h>

/* for array indexing */
#define FEMUR 0
#define TIBIA 1
#define FOOT  2
#define X     0
#define Y     1
#define Z     2

/* general */
const double WATER_DENSITY = 1000.0;    /* [kg/m³] */
const double GRAVITY       = 9.81;      /* [m/s²] */

/* some arithmetics */
#define SIGN(x)                 ((x)<0.0?-1.0:1.0)
#define SQ(x)                   ((x)*(x))
#define CYLINDER_VOLUME(r, h)   (M_PI*SQ(r)*(h))

/* constants that depend on salamander model in salamander.wbt file */
const double FEMUR_LENGTH    = 0.024;
const double FEMUR_RADIUS    = 0.00575;
const double TIBIA_LENGTH    = 0.045;
const double TIBIA_RADIUS    = 0.00575;
const double FOOT_RADIUS     = 0.013;
const double FOOT_HEIGHT     = 0.0075;
const double WATER_LEVEL     = 0.0;
const double HIP_LENGTH      = 0.008;
const double HIP_RADIUS      = 0.00975;
const double SEGMENT_HEIGHT  = 0.056;
const double SEGMENT_LENGTH  = 0.095;
const double SEGMENT_WIDTH   = 0.038;
#define NUM_SEGMENTS         9
#define SEGMENT_WIDTH_HALF   (SEGMENT_WIDTH / 2.0)
#define SEGMENT_CYL_VOLUME   (M_PI * SQ(SEGMENT_WIDTH_HALF) * SEGMENT_HEIGHT)
#define SEGMENT_BOX_VOLUME   (SEGMENT_LENGTH * SEGMENT_WIDTH * SEGMENT_HEIGHT)
#define SEGMENT_TOTAL_VOLUME (SEGMENT_CYL_VOLUME + SEGMENT_BOX_VOLUME)

/* drag coefficients (lambda = 0.5 * rho_water * Surface_perp * C_drag) */
const double SEGMENT_CX = 2.566666;    /* depends on segment geometry */
const double SEGMENT_CY = 1.95;        /* depends on segment geometry */
#define LAMBDA_X  (0.5 * WATER_DENSITY * SEGMENT_LENGTH * SEGMENT_HEIGHT * SEGMENT_CX)
#define LAMBDA_Y  (0.5 * WATER_DENSITY * SEGMENT_LENGTH * SEGMENT_WIDTH * SEGMENT_CY)

typedef struct leg_element_info {
  dBodyID body;
  double volume;                            /* used for computation of Archimedes force */
  double cx, cy, cz;                        /* drag coefficients */
  double sx, sy, sz;                        /* surface perpendicular to axis */
  double lambda_x, lambda_y, lambda_z;      /* lambda_x = rho * sx * cx */
} LEG_ELEMENT_INFO;

typedef struct leg_info {
  LEG_ELEMENT_INFO leg_element[3];          /* FEMUR, TIBIA, FOOT */
} LEG_INFO;

/* global variables */
static LEG_INFO leg[4];                            /* array of legs */
static dBodyID segments_bodies[NUM_SEGMENTS];      /* array of salamander bodies */
static int physics_enabled = 1;                    /* disable this plugin if geoms or bodies are not found */
static int i;

/* general ODE objects that will be provided by Webots */
dWorldID world = NULL;
dSpaceID space = NULL;
dJointGroupID contact_joint_group = NULL;

dBodyID getBody(const char *def) {
  dBodyID body = dWebotsGetBodyFromDEF(def);
  if (! body) {
    dWebotsConsolePrintf("Warning: did not find dGeom for DEF %s Servo\n", def);
    dWebotsConsolePrintf("Physics plugin will be disabled ...\n");
    physics_enabled = 0;
  }
  return body;
}

/* function invoked by Webots */
void webots_physics_init(dWorldID w, dSpaceID s, dJointGroupID j) {

  /* stores Webots supplied objects */
  world = w;
  space = s;
  contact_joint_group = j;

  /* get pointers to all body servos (geoms and bodies) */
  for (i = 0; i < NUM_SEGMENTS; i++) {
    /* the first element is a Robot node and not a Servo node */
    char servo_name[128];
    if (i == 0)
      sprintf(servo_name, "SALAMANDER");
    else
      sprintf(servo_name, "SERVO_%d", i);

    segments_bodies[i] = getBody(servo_name);
  }

  /* get pointers to all leg servos (geoms and bodies) */
  for (i = 0; i < 4; i++) {
    char servo_name[128];
    sprintf(servo_name, "SERVO_LEG_%d", i + 1);
    leg[i].leg_element[TIBIA].body = getBody(servo_name);

    sprintf(servo_name, "SERVO_TIBIA_%d", i + 1);
    leg[i].leg_element[FEMUR].body = getBody(servo_name);

    leg[i].leg_element[FEMUR].volume =
      CYLINDER_VOLUME(HIP_RADIUS, HIP_LENGTH) +
      CYLINDER_VOLUME(FEMUR_RADIUS, FEMUR_LENGTH - HIP_LENGTH);

    leg[i].leg_element[TIBIA].volume =
      CYLINDER_VOLUME(TIBIA_RADIUS, TIBIA_LENGTH) +
      CYLINDER_VOLUME(FOOT_RADIUS, FOOT_HEIGHT);

    /* compute drag coefficients: for the femur and the tibia we don't compute */
    /* the forces perpendicular to the cylinder caps (cx == 0) */
    leg[i].leg_element[FEMUR].cx = 0.0;
    leg[i].leg_element[FEMUR].cy = 0.24;     /* sphere */
    leg[i].leg_element[FEMUR].cz = 0.24;

    leg[i].leg_element[TIBIA].cx = 0.0;      /* none */
    leg[i].leg_element[TIBIA].cy = 0.24;
    leg[i].leg_element[TIBIA].cz = 0.24;

    leg[i].leg_element[FOOT].cx = 1.32;      /* disc */
    leg[i].leg_element[FOOT].cy = 0.24;
    leg[i].leg_element[FOOT].cz = 0.24;

    /* compute surfaces perpendicular to axes */
    /* femur perp face */
    leg[i].leg_element[FEMUR].sx = M_PI * SQ(FEMUR_RADIUS);

    /* parallel to face */
    leg[i].leg_element[FEMUR].sy = leg[i].leg_element[FEMUR].sz = FEMUR_LENGTH * 2 * FEMUR_RADIUS;

    /* tibia perp to face */
    leg[i].leg_element[TIBIA].sx = M_PI * SQ(TIBIA_RADIUS);

    /* tibia parallel to face */
    leg[i].leg_element[TIBIA].sz = leg[i].leg_element[TIBIA].sy = TIBIA_LENGTH * 2 * TIBIA_RADIUS;

    /* surface perp to cylinder cap */
    leg[i].leg_element[FOOT].sx = M_PI * SQ(FOOT_RADIUS);

    /* surface parallel to cylinder side */
    leg[i].leg_element[FOOT].sz = leg[i].leg_element[FOOT].sy = FOOT_HEIGHT * 2 * FOOT_RADIUS;

    /* compute lambdas (lambda_x = rho * sx * cx) */
    leg[i].leg_element[FEMUR].lambda_x = WATER_DENSITY *
        leg[i].leg_element[FEMUR].sx * leg[i].leg_element[FEMUR].cx;
    leg[i].leg_element[FEMUR].lambda_y = WATER_DENSITY *
        leg[i].leg_element[FEMUR].sy * leg[i].leg_element[FEMUR].cy;
    leg[i].leg_element[FEMUR].lambda_z = WATER_DENSITY *
        leg[i].leg_element[FEMUR].sz * leg[i].leg_element[FEMUR].cz;

    leg[i].leg_element[TIBIA].lambda_x = WATER_DENSITY *
        leg[i].leg_element[TIBIA].sx * leg[i].leg_element[TIBIA].cx;
    leg[i].leg_element[TIBIA].lambda_y = WATER_DENSITY *
        leg[i].leg_element[TIBIA].sy * leg[i].leg_element[TIBIA].cy;
    leg[i].leg_element[TIBIA].lambda_z = WATER_DENSITY *
        leg[i].leg_element[TIBIA].sz * leg[i].leg_element[TIBIA].cz;

    leg[i].leg_element[FOOT].lambda_x = WATER_DENSITY *
        leg[i].leg_element[FOOT].sx * leg[i].leg_element[FOOT].cx;
    leg[i].leg_element[FOOT].lambda_y = WATER_DENSITY *
        leg[i].leg_element[FOOT].sy * leg[i].leg_element[FOOT].cy;
    leg[i].leg_element[FOOT].lambda_z = WATER_DENSITY *
        leg[i].leg_element[FOOT].sz * leg[i].leg_element[FOOT].cz;
  }
}

/* apply hydrodynamic and hydrostatic forces */
void webots_physics_step()
{
  if (! physics_enabled) return;

  dReal f_archimede, lambda_z;

  for (i = 0; i < NUM_SEGMENTS; i++) {
    const dReal *lin_velocity = dBodyGetLinearVel(segments_bodies[i]);
    const dReal *position_vect = dBodyGetPosition(segments_bodies[i]);
    dVector3 speed;
    dBodyVectorFromWorld(segments_bodies[i],
                         lin_velocity[X],
                         lin_velocity[Y], lin_velocity[Z], speed);

    /* compute Archimedes force */
    dReal elevation = position_vect[Y];

    if ((elevation - WATER_LEVEL) >= (SEGMENT_HEIGHT / 2)) {
      /* then the robot is completely out of the water */
      f_archimede = 0;
    }
    else if (elevation > WATER_LEVEL && elevation - WATER_LEVEL <= SEGMENT_HEIGHT / 2) {
      /* then the robot has the bottom in the water */
      f_archimede = (SEGMENT_HEIGHT / 2 - (elevation - WATER_LEVEL))
          * SEGMENT_LENGTH * SEGMENT_WIDTH * WATER_DENSITY * GRAVITY;
    }
    else if (elevation <= WATER_LEVEL && WATER_LEVEL - elevation <= SEGMENT_HEIGHT / 2) {
      /* then the robot is almost completely in the water */
      f_archimede = (SEGMENT_HEIGHT / 2 + (WATER_LEVEL - elevation))
          * SEGMENT_LENGTH * SEGMENT_WIDTH * WATER_DENSITY * GRAVITY;
    }
    else
      f_archimede = SEGMENT_TOTAL_VOLUME * WATER_DENSITY * GRAVITY;

    /* add Archimedes force */
    dBodyAddForce(segments_bodies[i], 0, f_archimede, 0);

    if (elevation < WATER_LEVEL) {
      /* compute drag forces */
      /* the 3 first elements are special... */
      if (i == 0)
        lambda_z = 0.3;
      else if (i == 1)
        lambda_z = 0.2;
      else if (i == 2)
        lambda_z = 0.1;
      else
        lambda_z = 0.0;
    
      /* transversal */
      dReal f_x = -SIGN(speed[X]) * LAMBDA_X * SQ(speed[X]);

      /* up - down */
      dReal f_y = -SIGN(speed[Y]) * LAMBDA_Y * SQ(speed[Y]);

      /* longitudinal */
      dReal f_z = -SIGN(speed[Z]) * lambda_z * SQ(speed[Z]);
      
      /* apply drag forces */
      dBodyAddRelForce(segments_bodies[i], f_x, f_y, f_z);

      /* compute roll resistance */
      /* angular velocity in world coodinates */
      const dReal *avw = dBodyGetAngularVel(segments_bodies[i]);
        
      /* angular velocity in body relative coodinates */
      dVector3 avb;
      dBodyVectorFromWorld(segments_bodies[i], avw[X], avw[Y], avw[Z], avb);
        
      /* apply torque to damp the roll effect */
      dBodyAddRelTorque(segments_bodies[i], 0.0, 0.0, -avb[Z] * 0.005);
    }    
  }

  /* hydrodynamics on legs */
  for (i = 0; i < 4; i++) {

    /* get speed and position: TIBIA */
    const dReal *lin_velocity = dBodyGetLinearVel(leg[i].leg_element[TIBIA].body);
    const dReal *position_vect = dBodyGetPosition(leg[i].leg_element[TIBIA].body);
    dVector3 speed;
    dBodyVectorFromWorld(leg[i].leg_element[TIBIA].body,
                         lin_velocity[X], lin_velocity[Y], lin_velocity[Z], speed);

    dReal elevation = position_vect[Y];
    if (elevation < WATER_LEVEL) {
      /* compute and add Archimedes force */
      dReal f_archimede = leg[i].leg_element[TIBIA].volume * WATER_DENSITY * GRAVITY;
      dBodyAddForce(leg[i].leg_element[TIBIA].body, 0, f_archimede, 0);
 
      /* compute drag forces on tibia */
      /* x: transversal, y: up - down, z: longitudinal */
      dReal f_x = -SIGN(speed[X]) * (leg[i].leg_element[TIBIA].lambda_x +
                  leg[i].leg_element[FOOT].lambda_x) * SQ(speed[X]);
      dReal f_y = -SIGN(speed[Y]) * (leg[i].leg_element[TIBIA].lambda_y +
                  leg[i].leg_element[FOOT].lambda_y) * SQ(speed[Y]);
      dReal f_z = -SIGN(speed[Z]) * (leg[i].leg_element[TIBIA].lambda_z +
                  leg[i].leg_element[FOOT].lambda_z) * SQ(speed[Z]);
      dBodyAddRelForce(leg[i].leg_element[TIBIA].body, f_x, f_y, f_z);
    }

    /* get speed and position: FEMUR */
    lin_velocity = dBodyGetLinearVel(leg[i].leg_element[FEMUR].body);
    position_vect = dBodyGetPosition(leg[i].leg_element[FEMUR].body);
    dBodyVectorFromWorld(leg[i].leg_element[FEMUR].body,
                         lin_velocity[X], lin_velocity[Y], lin_velocity[Z], speed);

    if (elevation < WATER_LEVEL) {
      /* add Archimedes force vertically */
      f_archimede = leg[i].leg_element[FEMUR].volume * WATER_DENSITY * GRAVITY;
      dBodyAddForce(leg[i].leg_element[FEMUR].body, 0, f_archimede, 0);

      /* compute drag forces on femur */
      /* x: transversal, y: up - down, z: longitudinal */
      dReal f_x = -SIGN(speed[X]) * leg[i].leg_element[FEMUR].lambda_x * SQ(speed[X]);
      dReal f_y = -SIGN(speed[Y]) * leg[i].leg_element[FEMUR].lambda_y * SQ(speed[Y]);
      dReal f_z = -SIGN(speed[Z]) * leg[i].leg_element[FEMUR].lambda_z * SQ(speed[Z]);
      dBodyAddRelForce(leg[i].leg_element[FEMUR].body, f_x, f_y, f_z);
    }
  }
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
  return 0;  /* collision not handled, otherwise should return 1 */
}

void webots_physics_cleanup() {
  /* nothing to cleanup */
}
