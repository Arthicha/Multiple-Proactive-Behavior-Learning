//
// created by zumo arthicha
// (zumoarthicha@gmail.com)
//

#include "planner.h"

#define UI 0

using namespace std;

planner::planner(int argc, char *argv[]) {
	
	/* ros initialization */
	int _argc = 0;
	char** _argv = nullptr;
	ros = new rosClass(_argc, _argv);	
	ros->getRosParam();
				
	this->running = 1;
	this->drawing = 0;
	this->goal.x = 0;
	this->goal.y = 0;
	this->currentIdx = 0;
	this->pointCloud = createPointCloud(100);
	this->velocity.linearVelocity = 0.0;
	this->velocity.angularVelocity = 0.0;
	this->pose.point.x = 0.0;
	this->pose.point.y = 0.0;
	this->pose.yaw = 0.0;
	float d = 0.4;
	this->rect.xmin = -1.0*d;
	this->rect.ymin = -1.0*d;
	this->rect.xmax = +1.0*d;
	this->rect.ymax = +1.0*d;

	this->config.maxSpeed = 1.5; //1.5
	this->config.minSpeed = -1.5; // -1.5
	this->config.maxYawrate = 40.0* M_PI / 180.0;
	this->config.maxAccel = 1500.0;
	this->config.maxdYawrate = 11000.0 * M_PI / 180.0;
	this->config.velocityResolution = 3*0.05;//0.05 vs 0.02;
	this->config.yawrateResolution = 0.5*0.5 * M_PI / 180.0;
	this->config.predictTime = 1.0*3;//*3;
	this->config.dt = 0.05*2;//*2;
	this->config.heading = 10;
	this->config.clearance = 10/10;
	this->config.velocity = 0.0; // dynamics use 0.1
	this->config.base = rect;


	
	if (UI)
	{
		SDL_WM_SetCaption("test", NULL);
		SDL_SetVideoMode(RESOLUTION_WIDTH, RESOLUTION_HEIGHT, 0, SDL_OPENGL);
		this->initGL();	 
	}
	freePointCloud(this->pointCloud);
			this->pointCloud = createPointCloud(100);
			this->currentIdx = 0;
			this->pose.point.x = 0.0;
			this->pose.point.y = 0.0;
			this->pose.yaw = 0.0;
			this->velocity.linearVelocity = 0.0;
			this->velocity.angularVelocity = 0.0;

	//SDL_Quit();



} // constructor



bool planner::update()
{
	float S = 50;
  if (this->running)
  {
  	if (UI)
  	{
  		SDL_Event sdlEvent;
		glClear(GL_COLOR_BUFFER_BIT);
  	}
	
  		// update rosparam
  		this->config.maxSpeed = (ros->max_speed);
		this->config.minSpeed = -1.0*ros->max_speed;
		this->config.velocityResolution = ros->speed_resolution;
		this->config.yawrateResolution = ros->yaw_resolution*M_PI / 180.0;
		this->config.predictTime = ros->predict_time;
		this->config.dt = ros->dt;
		this->config.heading = ros->heading_cost;
		this->config.clearance = ros->clearance_cost;
		this->config.velocity = ros->velocity_cost;
		this->detectionlength_gain = ros->detectionlength_gain;
    	this->detectionangle_gain = ros->detectionangle_gain;
    	this->detectionangle_offset = ros->detectionangle_offset;
		  
		this->goal.x = ros->getGoal(0);
		this->goal.y = ros->getGoal(1);
		
		
		float theta = 0;
		float d = 0;
		float yaw = 0;
		int cnt = 0;
		this->localCloud = createPointCloud(ros->getLidarSize()+ros->detectionsx.size());
		if (UI) glColor3f(1, 0, 0);
		
		for (int ll=0;ll<ros->getLidarSize();ll++)
		{
			theta = ros->getLidarAngle(ll)*3.14/180;
			yaw = this->ros->getRobotPose(2);
			//theta += (90+30)*3.14/180;
			theta += (90+90)*3.14/180;
			d = ros->getLidarDistance(ll);
			if (d >= ros->sensor_limit) d = 100;
			this->localCloud->points[ll].x = this->pose.point.x + d*cos(-theta+yaw);
			this->localCloud->points[ll].y = this->pose.point.y + d*sin(-theta+yaw);
			if (UI) this->drawCircle(S*(this->localCloud->points[ll].x), S*(this->localCloud->points[ll].y), 2);	
		}


		//this->velocity.linearVelocity = 0.0;
		//this->velocity.angularVelocity = 0.0;
		this->tmpPointCloud = createPointCloud(ros->getLidarSize());
		//memcpy(this->tmpPointCloud->points, this->localCloud->points, this->currentIdx*sizeof(Point));
		this->velocity = planning(this->pose, this->velocity, this->goal, this->localCloud, this->config,ros->sensor_limit,ros->coarse_step);
		
		if (UI) glColor3f(0, 1, 0);
		
		float newgain = 5;
		this->pose_tmp = motion(this->pose, this->velocity, this->config.predictTime/2);
		this->pose_tmp2 = motion(this->pose, this->velocity, this->config.predictTime);
		ros->publishExpectation(this->pose_tmp.point.x,this->pose_tmp.point.y,this->pose_tmp.yaw,this->pose_tmp2.point.x,this->pose_tmp2.point.y,this->pose_tmp2.yaw);

		if (UI) this->drawCircle(S*(this->pose_tmp.point.x), S*(this->pose_tmp.point.y), 1);
		if (UI) this->drawCircle(S*(this->pose_tmp2.point.x), S*(this->pose_tmp2.point.y), 1);

		this->pose = motion(this->pose, this->velocity, this->config.predictTime);
		if (UI) this->drawCircle(S*(this->pose.point.x), S*(this->pose.point.y), 2);

		this->pose.point.x = this->ros->getRobotPose(0);
		this->pose.point.y = this->ros->getRobotPose(1);
		this->pose.yaw = this->ros->getRobotPose(2);
		if (UI) 
		{
			glColor3f(0, 0, 1);
			this->drawCircle(S*this->goal.x, S*this->goal.y, 5);
			glColor3f(1, 1, 1);
			this->drawCircle(S*this->pose.point.x, S*this->pose.point.y, 3);
			this->drawCircle(S*this->pose.point.x+0.2*S*cos(this->pose.yaw), S*this->pose.point.y+0.2*S*sin(this->pose.yaw), 3);
			this->drawCircle(S*this->pose.point.x-0.1*S*cos(this->pose.yaw), S*this->pose.point.y-0.1*S*sin(this->pose.yaw), 3);
		}

		

		
		freePointCloud(this->localCloud);
		if (UI) SDL_GL_SwapBuffers();

		/*float dx = this->goal.x-this->pose.point.x;
		float dy = this->goal.y-this->pose.point.y;

		float diff =  sqrtf(dx*dx+dy*dy);

		if (diff < 0.3) return true;*/
	}
	return false;
}

	/*
	NOTE: 1 Unit = 0.1 m
	That's why everything is multiplied or divided by 10.
	*/

	void planner::initGL(void) {
	  glMatrixMode(GL_PROJECTION);
	  glLoadIdentity();
	  glOrtho(-RESOLUTION_WIDTH/2, RESOLUTION_WIDTH/2,
	          -RESOLUTION_HEIGHT/2, RESOLUTION_HEIGHT/2,
	          1, -1);
	  glMatrixMode(GL_MODELVIEW);
	  glLoadIdentity();
	}

	void planner::drawCircle(GLfloat x, GLfloat y, GLfloat radius){
	  int nTriangle = 20;
	  glBegin(GL_TRIANGLE_FAN);
	  glVertex2f(x, y);
	  for (int i = 0; i <= nTriangle; i++) { 
	    glVertex2f(x + (radius * cos(i * M_PI2 / nTriangle)), 
	               y + (radius * sin(i * M_PI2 / nTriangle)));
	  }
	  glEnd();
	}

	void planner::renderPointCloud(PointCloud *pointCloud, int size){
	  glBegin(GL_POINTS);
	  for (size_t i = 0; i < size; i++){
	    glColor3f(0, 0, 1);
	    this->drawCircle(pointCloud->points[i].x*10, pointCloud->points[i].y*10, 3.0);
	  }
	  glEnd();
	}

	void planner::renderRectangle(Rect rect, Pose pose) {
	  glPushMatrix();
	  glTranslatef(pose.point.x*10.0,
	               pose.point.y*10.0,
	               0.0f);
	  glRotatef(pose.yaw * 180.0/M_PI, 0, 0, 1);
	  glBegin(GL_POLYGON);
	  glColor3f(1, 0, 0);
	  glVertex2f(rect.xmin*10, rect.ymax*10);
	  glVertex2f(rect.xmin*10, rect.ymin*10);
	  glVertex2f(rect.xmax*10, rect.ymin*10);
	  glVertex2f(rect.xmax*10, rect.ymax*10);
	  glEnd();
	  glPopMatrix();
	}

bool planner::rosUpdate()
{
	this->ros->publishVelocityCommand(this->velocity.linearVelocity,this->velocity.angularVelocity);
	this->ros->publishIteration();
	return this->ros->rosStep();
}