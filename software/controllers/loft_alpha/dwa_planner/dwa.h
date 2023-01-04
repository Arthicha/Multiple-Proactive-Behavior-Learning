#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <fstream>
#include <math.h>
#include <string>
#include <map>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#define ROBOT_RADIUS 0.4

/*#ifndef _WIN32
#define max(a,b)				\
  ({ __typeof__ (a) _a = (a);			\
    __typeof__ (b) _b = (b);			\
    _a > _b ? _a : _b; })

#define min(a,b)				\
  ({ __typeof__ (a) _a = (a);			\
    __typeof__ (b) _b = (b);			\
    _a < _b ? _a : _b; })
#endif*/

using namespace std;

typedef struct {
  float xmin;
  float ymin;
  float xmax;
  float ymax;
} Rect;

typedef struct {
  float maxSpeed;
  float minSpeed;
  float maxYawrate;
  float maxAccel;
  float maxdYawrate;
  float velocityResolution;
  float yawrateResolution;
  float dt;
  float predictTime;
  float heading;
  float clearance;
  float velocity;
  Rect base;
} Config;

typedef struct {
  float linearVelocity;
  float angularVelocity;
} Velocity;

typedef struct {
  float x;
  float y;
} Point;

typedef struct {
  int size;
  Point *points;
} PointCloud;

typedef struct {
  Point point;
  float yaw;
} Pose;

typedef struct {
  int nPossibleV;
  float *possibleV;
  int nPossibleW;
  float *possibleW;
} DynamicWindow;



void
createDynamicWindow(Config config, DynamicWindow **dynamicWindow);
Pose motion(Pose pose, Velocity velocity, float dt);
float calculateVelocityCost(Velocity velocity,  Velocity velocity_x);
float calculateHeadingCost(Pose pose, Point goal);
float
calculateClearanceCost
(Pose pose, Velocity velocity, PointCloud *pointCloud, Config config,float max_d, float T);
Velocity
planning
(Pose pose, Velocity velocity, Point goal, PointCloud *pointCloud, Config config, float max_d, int coarse_step);
PointCloud* createPointCloud(int size);
void freePointCloud(PointCloud* pointCloud);
void freeDynamicWindow(DynamicWindow *dynamicWindow);
