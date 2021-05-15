#include <Eigen/LU>
#include <iostream>

#include <stdio.h>
#include <unistd.h>

#include "../simulator/constants.h"
#include "../simulator/graphics.h"
#include "../simulator/utils.h"
#include "../simulator/world_interface.h"
#include "plan.h"
#include "search.h"

using namespace NavSim;
const double CONTROL_HZ = MAX_SPEED / PLAN_RESOLUTION;
const double REFRESH_HZ = 30;
const int REFRESH_PER_CONTROL = REFRESH_HZ / CONTROL_HZ;

void drawPlan(MyWindow &plan_window, const transform_t &plan_odom,
    const plan_t &p, const point_t &goal, const points_t &lidar_hits,
    const points_t &landmarks)
{
  trajectory_t traj({});
  transform_t trans = toTransform({0,0,0});
  traj.push_back(trans);
  for (int i = 0; i < p.rows(); i++)
  {
    double theta = p(i,0);
    double x = p(i,1);
    // Doesn't matter if we rotate first because our plans never rotate
    // and move forward at the same time.
    trans = toTransformRotateFirst(x, 0, theta) * trans;
    traj.push_back(trans);
  }
  transform_t base = toTransform({DEFAULT_WINDOW_CENTER_X,DEFAULT_WINDOW_CENTER_Y,M_PI/2});
  transform_t plan_base = plan_odom * readOdom().inverse() * base;
  plan_window.drawTraj(transformTraj(traj, plan_base), sf::Color::Red);
  plan_window.drawPoints(transformReadings({goal}, plan_base), sf::Color::Green, 10);
  plan_window.drawPoints(transformReadings(lidar_hits, plan_base), sf::Color::Red, 3);
  plan_window.drawPoints(transformReadings(landmarks, plan_base), sf::Color::Blue, 4);
  plan_window.drawRobot(base, sf::Color::Black);
  plan_window.display();
}

int main()
{
  world_interface_init();
  MyWindow plan_window("Planning visualization");
  URCLeg leg = getLeg(5);
  setGoal(leg);
  std::cout << "Type 'q' to quit, 'wasd' to move around, 'r' to toggle autonomous mode.\n";

  bool autonomous = false;

  int c = 0;
  int iter = 0;
  bool done = false;
  point_t waypoint;
  transform_t plan_odom;
  points_t lidar_hits;
  points_t landmarks;
  plan_t plan;

  while (true) {
    while ((c = plan_window.pollWindowEvent()) != -1) {
      switch (c) {
        case 22:
        case 73:
          setCmdVel(0.0, MAX_SPEED);
          break;
        case 18:
        case 74:
          setCmdVel(0.0, -MAX_SPEED);
          break;
        case 0:
        case 71:
          setCmdVel(0.5 * MAX_SPEED / ROBOT_WHEEL_BASE, 0.0);
          break;
        case 3:
        case 72:
          setCmdVel(-0.5 * MAX_SPEED / ROBOT_WHEEL_BASE, 0.0);
          break;
        case -3:
          setCmdVel(0.0, 0.0);
          break;
        case 17:
          autonomous = !autonomous;
          break;
        case -2:
          done = true;
          break;
        default:
          break;
      }
    }
    if (done) break;

    if (iter++ % REFRESH_PER_CONTROL == 0) {
      plan_odom = readOdom();
      lidar_hits = readLidarScan();
      landmarks = readLandmarks();
      if (autonomous) setCmdVel(0.,0.); // In case planning takes a long time
      plan = act(lidar_hits, landmarks, &waypoint);
      if (autonomous) {
        if (plan.size() == 0)
        {
          std::cout << "Disabling autonomous mode.\n";
          autonomous = false;
          setCmdVel(0.,0.);
        }
        else
        {
          action_t action = plan.row(0);
          setCmdVel(action(0)*CONTROL_HZ, action(1)*CONTROL_HZ);
        }
      }
    }
    drawPlan(plan_window, plan_odom, plan, waypoint, lidar_hits, landmarks);

    usleep(1000*1000 / REFRESH_HZ);
  }

  return 0;
}
