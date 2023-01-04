//
// developed by zumo arthicha
// email: zumoarthicha@gmail.com
// update: 20 Jun 2021
//

#ifndef PLANNER_H
#define PLANNER_H

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <random>
#include <iomanip>
#include <unistd.h>
#include <fstream>
#include <math.h>
#include <string>
#include <map>
#include "SDL/SDL.h"
#include <GL/gl.h>
#include "dwa.h"
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float32MultiArray.h"
#include "rosClass.h"
#include<unistd.h>

#define RESOLUTION_WIDTH  800
#define RESOLUTION_HEIGHT 600
#define M_PI 3.14159265358979323846
#define M_PI2 M_PI * 2.0
#define ROBOT_D 1.0

using namespace std;
using std::cout;
using std::endl;
using std::setprecision;


class planner{
public:
    planner(int argc, char *argv[]);
    bool update();
    bool rosUpdate();
    

private:

    void initGL(void);
    void drawCircle(GLfloat x, GLfloat y, GLfloat radius);
    void renderPointCloud(PointCloud *pointCloud, int size);
    void renderRectangle(Rect rect, Pose pose);


    rosClass * ros;
    DynamicWindow *dw;

    int running;
    int drawing;
    Point goal;
    PointCloud *tmpPointCloud;
    PointCloud *pointCloud;
    PointCloud *localCloud;
    int currentIdx;

    float detectionlength_gain = 1.0;
    float detectionangle_gain = 1.0;
    float detectionangle_offset = 0.0;

    Velocity velocity;
    Velocity velocity_tmp;
    Pose pose;
    Pose pose_tmp;
    Pose pose_tmp2;
  

    Config config;
    Rect rect;



    

    

};

#endif //PLANNER_H
