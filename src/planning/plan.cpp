
#include "plan.h"

#include <Eigen/Core>
#include <iostream>
#include <queue>
#include <set>

#include "../simulator/constants.h"
#include "../simulator/graphics.h"
#include "../simulator/world_interface.h"

using namespace NavSim;

// TODO implement goal orientations?
double heuristic(int x, int y, const point_t &goal)
{
  double dx = goal(0) - x*PLAN_RESOLUTION;
  double dy = goal(1) - y*PLAN_RESOLUTION;
  return sqrt(dx*dx + dy*dy);
}

struct Node
{
  int x;
  int y;
  int theta; // allows values 0..7; gives angle in increments of pi/4
  double acc_cost; // accumulated cost
  double heuristic_to_goal;
  int acc_steps; // for knowing how long (in discrete steps) the plan will be
  struct Node *parent;
  // action: the action taken to reach this node from the parent node
  // Tracking this makes it easier to construct the plan later.
  action_t action;

  // Note: In the robot frame, starting pose is always (0,0,0).
  Node(Node *p, action_t &a, const point_t &goal) : x(0), y(0), theta(0),
        acc_cost(0.), heuristic_to_goal(0.), acc_steps(0),
        parent(p), action(a)
  {
    if (p != nullptr)
    {
      acc_steps = p->acc_steps + 1;
      bool moving_forward = action(1) > 0.0;
      double step_cost = moving_forward ? action(1) : RADIAN_COST * abs(action(0));
      acc_cost = p->acc_cost + step_cost;
      if (moving_forward)
      {
        theta = p->theta;
        int dx(0), dy(0);
        if (theta >= 1 && theta <= 3) dy = 1;
        if (theta >= 3 && theta <= 5) dx = -1;
        if (theta >= 5 && theta <= 7) dy = -1;
        if (theta == 7 || theta <= 1) dx = 1;
        x = p->x + dx;
        y = p->y + dy;
      }
      else
      {
        x = p->x;
        y = p->y;
        theta = (p->theta + ((action(0) > 0.0) ? 1 : 7)) % 8;
      }
    }
    heuristic_to_goal = heuristic(x, y, goal);
  }
};

std::ostream& operator<< (std::ostream &out, const Node &n)
{
  out <<"("<< n.x << " " << n.y << " " << n.theta <<") ";
  return out;
}

class NodeEqualityCompare
{
public:
  bool operator() (const Node *lhs, const Node *rhs) const
  {
    bool res = lhs->x <  rhs->x ||
               (lhs->x == rhs->x && lhs->y <  rhs->y) ||
               (lhs->x == rhs->x && lhs->y == rhs->y && lhs->theta < rhs->theta);
    return res;
  }
};

class NodeCompare
{
public:
  bool operator() (const Node *lhs, const Node *rhs)
  {
    return (lhs->acc_cost + EPS*(lhs->heuristic_to_goal)) >
           (rhs->acc_cost + EPS*(rhs->heuristic_to_goal));
  }
};

using pqueue_t = std::priority_queue<Node*, std::vector<Node*>, NodeCompare>;
using set_t = std::set<Node*, NodeEqualityCompare>;

bool is_valid(const Node *n, const points_t &lidar_hits)
{
  // To get away from obstacles, turning is always allowed.
  if (n->action(1) == 0.0) return true;
  pose_t p(n->x * PLAN_RESOLUTION, n->y * PLAN_RESOLUTION, n->theta * M_PI/4);
  transform_t tf = toTransform(p);
  return !collides(tf, lidar_hits, SAFE_RADIUS);
}

// Goal given in robot frame
plan_t getPlan(const points_t &lidar_hits, const point_t &goal, double goal_radius)
{
  std::cout << "Planning... " << std::flush;
  action_t action = action_t::Zero();
  std::vector<Node*> allocated_nodes;
  Node *start = new Node(nullptr, action, goal);
  allocated_nodes.push_back(start);
  pqueue_t fringe; // If you haven't guessed, we'll be using A*
  set_t visited_set;
  fringe.push(start);
  visited_set.insert(start);
  Node *n = start;
  plan_t valid_actions(3,2);
  valid_actions << 0., 0., M_PI/4, 0., -M_PI/4, 0.;
  int counter = 0;
  bool success = false;
  while (fringe.size() > 0)
  {
    if (counter++ > MAX_ITERS)
    {
      break;
    }
    n = fringe.top();
    fringe.pop();
    if (n->heuristic_to_goal < goal_radius)
    {
      success = true;
      break;
    }
    else
    {
      double forward_dist = (n->theta % 2 == 1) ? PLAN_RESOLUTION*1.414 : PLAN_RESOLUTION;
      valid_actions(0,1) = forward_dist;
      for (int i = 0; i < valid_actions.rows(); i++)
      {
        action = valid_actions.row(i);
        Node *next = new Node(n, action, goal);
        allocated_nodes.push_back(next);
        if (is_valid(next, lidar_hits) && visited_set.count(next) == 0)
        {
          visited_set.insert(next);
          fringe.push(next);
        }
      }
    }
  }
  if (success == false) n = start;

  plan_t plan(n->acc_steps,2);
  for (int i = n->acc_steps-1; i >= 0; i--)
  {
    plan.row(i) = n->action;
    n = n->parent;
  }

  std::cout << "finished in " << counter << " iterations (";
  std::cout << allocated_nodes.size() << " visited nodes)\n";
  for (Node *p : allocated_nodes)
  {
    free(p);
  }

  return plan;
}

double planCostFromIndex(plan_t &plan, int idx) {
  double cost = 0;
  for (int i = idx; i < plan.rows(); i++) {
    action_t action = plan.row(i);
    cost += abs(action(1)) + RADIAN_COST * abs(action(0));
  }
  return cost;
}

