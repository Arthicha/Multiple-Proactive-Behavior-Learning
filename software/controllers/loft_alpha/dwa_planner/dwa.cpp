#include "dwa.h"

#define PI 3.14159265

void
createDynamicWindow(Velocity velocity, Config config, DynamicWindow **dynamicWindow) {
  float minV = 0.0;
  float maxV = 0.0;
  float minW = 0.0;
  float maxW = 0.0;
  
  maxV = config.maxSpeed;
  minV = config.minSpeed;
  minW = -config.maxYawrate;
  maxW = config.maxYawrate;

  int nPossibleV = (maxV - minV) / config.velocityResolution;
  nPossibleV++;
  int nPossibleW = (maxW - minW) / config.yawrateResolution;
  nPossibleW++;
  *dynamicWindow = (DynamicWindow*) malloc(sizeof(DynamicWindow));

  (*dynamicWindow)->possibleV = (float*) malloc(nPossibleV * sizeof(float));
  (*dynamicWindow)->possibleW = (float*) malloc(nPossibleW * sizeof(float));
  (*dynamicWindow)->nPossibleV = nPossibleV;
  (*dynamicWindow)->nPossibleW = nPossibleW;

  for(int i=0; i < nPossibleV; i++) {

    (*dynamicWindow)->possibleV[i] = minV + (float)i * config.velocityResolution;

    
  }

  for(int i=0; i < nPossibleW; i++) {
    (*dynamicWindow)->possibleW[i] = minW + (float)i * config.yawrateResolution;
  }
}

void freeDynamicWindow(DynamicWindow *dynamicWindow){
  free(dynamicWindow->possibleV);
  free(dynamicWindow->possibleW);
  free(dynamicWindow);
}

PointCloud* createPointCloud(int size){
  PointCloud* pointCloud = (PointCloud*) malloc(sizeof(PointCloud));
  pointCloud->points = (Point*) malloc(size * sizeof(Point));
  pointCloud->size = size;
  return pointCloud;
}

void freePointCloud(PointCloud* pointCloud){
  free(pointCloud->points);
  free(pointCloud);
}

Pose motion(Pose pose, Velocity velocity, float dt){
  Pose new_pose;
  new_pose.yaw = pose.yaw + velocity.angularVelocity * dt;
  new_pose.point.x = pose.point.x + velocity.linearVelocity * cos(new_pose.yaw) * dt;
  new_pose.point.y = pose.point.y + velocity.linearVelocity * sin(new_pose.yaw) * dt;
  return new_pose;
}

float calculateVelocityCost(Velocity velocity, Velocity velocity_x) {
  
  //if (d < 0.1) d -= 1000;
  //return 0 ;
  float dv = fabs(velocity.angularVelocity-velocity_x.angularVelocity)+fabs(velocity.linearVelocity-velocity_x.linearVelocity);
  //return - 0.001*fabs(velocity.linearVelocity) + 10*dv;
  return dv;
  //return fabs(velocity.angularVelocity-velocity_x.angularVelocity)+fabs(velocity.linearVelocity-velocity_x.linearVelocity);
}

float calculateHeadingCost(Pose pose, Point goal) {
  

  float dx = goal.x - pose.point.x;
  float dy = goal.y - pose.point.y;
  //float angleError = atan(dy/dx);
  //float angleCost = angleError - atan(sin(pose.yaw)/cos(pose.yaw));
  //fabs(atan2(sin(pose.yaw), cos(pose.yaw)))+

  //return 0.01*fabs(atan2(sin(angleCost), cos(angleCost))) + d;

  return sqrtf(dx*dx+dy*dy) ;

  /*float dx = goal.x - pose.point.x;
  float dy = goal.y - pose.point.y;
  float angleError = atan2(dy, dx);
  float angleCost = angleError - pose.yaw;
  //fabs(atan2(sin(angleCost), cos(angleCost)))+
  float d = sqrtf(dx*dx+dy*dy) + 0.001*sqrtf(angleCost*angleCost);
  //if (d < 0.1) d -= 1000;
  return d ;*/
}

float
calculateClearanceCost
(Pose pose, Velocity velocity, PointCloud *pointCloud, Config config,float max_d, float T) {

  Pose pPose = pose;
  float time = 0.0;
  float minr = 100000;
  float max_pen_obs = 0; 
  float r;
  float dx;
  float dy;

  float x;
  float y;
  float sumr = 0.0;
  bool hit = false;
  while (time < config.predictTime) {

    if ((not hit))
    {
      pPose = motion(pPose, velocity, config.dt);

      minr = 100000;

      for(int i = 0; i < pointCloud->size; ++i) 
      {
        dx = pPose.point.x - pointCloud->points[i].x;
        dy = pPose.point.y - pointCloud->points[i].y;
        r = dx*dx+dy*dy;
        if (r < minr){
          minr = r;
        }
      }
      if (minr <= (ROBOT_RADIUS*ROBOT_RADIUS))//0.4 vs 0.5)
      {
        sumr += 10000;//*config.dt
      }
    }
    time += config.dt;
  }
  return sumr;
}

Velocity
planning(Pose pose, Velocity velocity, Point goal,
         PointCloud *pointCloud, Config config, float max_d, int coarse_step) {

  DynamicWindow *dw;
  createDynamicWindow(velocity, config, &dw);
  Velocity pVelocity;
  Pose pPose = pose;
  float total_cost = FLT_MAX;
  float cost;
  Velocity bestVelocity;


  float T = config.predictTime;

  float goal_error = 0.0;
  float velocity_error = 0.0;
  float obstacle_error = 0.0;

  int ibest = 0;
  int jbest = 0;
  int step = coarse_step;

  // perform coarse hierachical DWA optimization
  for (int i = 0; i < (dw->nPossibleV); i+=step) {
    pVelocity.linearVelocity = dw->possibleV[i];
    for (int j = 0; j < dw->nPossibleW; j+=step) {
      pPose = pose;  
      pVelocity.angularVelocity = dw->possibleW[j];
      pPose = motion(pPose, pVelocity, T);
      
      obstacle_error = calculateClearanceCost(pose, pVelocity,pointCloud, config,max_d,T);
      if (obstacle_error < 10000)
      {
        goal_error = calculateHeadingCost(pPose, goal);
        velocity_error = calculateVelocityCost(pVelocity, velocity);
        cost = config.velocity * velocity_error + config.heading * goal_error + config.clearance * obstacle_error;
      }else{
        cost = config.clearance * obstacle_error;
      }
      
      if (cost <= total_cost) {
        total_cost = cost;
        bestVelocity = pVelocity;
        ibest = i;
        jbest = j;
      }
    }
  }

  
  total_cost = FLT_MAX;
  // perform fine hierachical DWA optimization
  int ii = 0;
  int jj = 0;
  for (int i = (ibest-step); i < (ibest+step+1); i+=1) {
    ii = i;
    if (ii < 0) ii = 0;
    if (ii > (dw->nPossibleV-1)) ii = dw->nPossibleV-1;
    pVelocity.linearVelocity = dw->possibleV[ii];
    for (int j = (jbest-step); j < (jbest+step+1); j+=1) {
      jj = j;
      if (jj < 0) jj = 0;
      if (jj > (dw->nPossibleW-1)) jj = dw->nPossibleW-1;
      pPose = pose;  
      pVelocity.angularVelocity = dw->possibleW[jj];
      pPose = motion(pPose, pVelocity, T);
      
      obstacle_error = calculateClearanceCost(pose, pVelocity,pointCloud, config,max_d,T);
      if (obstacle_error < 10000)
      {
        goal_error = calculateHeadingCost(pPose, goal);
        velocity_error = calculateVelocityCost(pVelocity, velocity);
        cost = config.velocity * velocity_error + config.heading * goal_error + config.clearance * obstacle_error;
      }else{
        cost = config.clearance * obstacle_error;
      }
      
      if (cost <= total_cost) {
        total_cost = cost;
        bestVelocity = pVelocity;
      }
    }
  }

  freeDynamicWindow(dw);
  return bestVelocity;
}
