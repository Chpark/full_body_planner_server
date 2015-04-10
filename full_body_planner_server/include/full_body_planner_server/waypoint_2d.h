#ifndef WAYPOINT_2D_H
#define WAYPOINT_2D_H

#include <vector>
#include <iostream>

namespace full_body_planner
{

struct Waypoint2D
{
    int frame;
    int agent_id;
    double radius;
    double x;
    double y;
    double orientation;
    int state;
    double pvx;
    double pvy;
    double vx;
    double vy;
    std::vector<int> neighbors;

    int locked;
    int failed;

    friend std::ostream& operator<<(std::ostream& os, const Waypoint2D& waypoint);
    friend std::istream& operator>>(std::istream& is, Waypoint2D& waypoint);
};

typedef std::vector<Waypoint2D> Trajectory2D;

}

#endif
