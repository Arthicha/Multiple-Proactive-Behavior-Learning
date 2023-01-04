#include "planner.h"

using namespace std;

int main(int argc, char * argv[]) {
  planner dwa_planner(argc,argv);
  bool running = true;

  random_device rd;
  default_random_engine eng(rd());
  uniform_real_distribution<> distr(-2, 2);
  float goalx = distr(eng);
  float goaly = distr(eng);
  bool reach = false;
  while(running)
  //for(int i=0;i<5;i++)
  {
    dwa_planner.update();
    running = dwa_planner.rosUpdate();
  }

  return 0;
}

