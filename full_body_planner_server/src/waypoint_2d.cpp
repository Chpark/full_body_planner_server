#include <full_body_planner_server/waypoint_2d.h>
#include <math.h>

using namespace std;

namespace full_body_planner
{

ostream& operator<<(ostream& os, const Waypoint2D& waypoint)
{
    double orientation = waypoint.orientation + M_PI_2;
    if (orientation > M_PI)
        orientation -= 2.0 * M_PI;

    os << waypoint.frame << " " << waypoint.agent_id << " " << waypoint.radius << " "
       << waypoint.x << " " << waypoint.y << " " << orientation << " " << waypoint.state << " "
       << waypoint.pvx << " " << waypoint.pvy << " " << waypoint.vx << " " << waypoint.vy << " [";
    for (int i = 0; i < waypoint.neighbors.size(); ++i)
        os << waypoint.neighbors[i] << " ";
    os << "] " << waypoint.locked << " " << waypoint.failed << std::endl;

    return os;
}

istream& operator>>(istream& is, Waypoint2D& waypoint)
{
    double state;

    is >> waypoint.frame >> waypoint.agent_id >> waypoint.radius >> waypoint.x >> waypoint.y >> waypoint.orientation >> state >> waypoint.pvx >> waypoint.pvy >> waypoint.vx >> waypoint.vy;

    waypoint.state = state;
    waypoint.orientation -= M_PI_2;
    if (waypoint.orientation <= -M_PI)
        waypoint.orientation += 2.0 * M_PI;

    waypoint.locked = 0;
    waypoint.failed = 0;

    return is;
}

}
