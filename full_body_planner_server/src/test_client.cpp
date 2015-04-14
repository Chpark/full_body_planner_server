#include <menge_3d_interface/3DInterfaceClient.h>
#include <menge_3d_interface/3DInterfaceMessageTypes.h>
#include <fstream>
#include <random>
#include <full_body_planner_server/waypoint_2d.h>

using namespace std;
using namespace full_body_planner;

static vector<Trajectory2D> readed_trajectories;
static int readed_index = 0;

const int NUM_WAYPOINTS = 20;

void readFile(const std::string& file_name, unsigned int& numAgents, unsigned int& numFrames, unsigned int& agentSize)
{
    const int NUM_NEIGHBORS = 1;

    // read from file
    std::ifstream trajectory_file;

    trajectory_file.open(file_name.c_str());
    if (trajectory_file.is_open())
    {
        Waypoint2D waypoint;
        while (trajectory_file >> waypoint)
        {
            if (readed_trajectories.size() <= waypoint.agent_id)
            {
                readed_trajectories.resize(waypoint.agent_id + 1);
            }

            waypoint.neighbors.resize(NUM_NEIGHBORS);
            waypoint.neighbors[0] = (waypoint.agent_id == 0) ? 1 : 0;

            readed_trajectories[waypoint.agent_id].push_back(waypoint);
        }
    }

    numAgents = readed_trajectories.size();
    numFrames = NUM_WAYPOINTS + 1;
    agentSize = 12 + NUM_NEIGHBORS;
}

bool getInput(vector<Trajectory2D>& trajectories)
{
    if (readed_index + 1 >= readed_trajectories[0].size())
        return false;

    int agent_size = readed_trajectories.size();
    trajectories.resize(agent_size);
    for (int i = 0; i < agent_size; ++i)
    {
        Trajectory2D& trajectory2d = trajectories[i];

        trajectory2d.resize(NUM_WAYPOINTS + 1);
        if (readed_index + NUM_WAYPOINTS + 1 <= readed_trajectories[i].size())
        {
            std::copy(readed_trajectories[i].begin() + readed_index,
                      readed_trajectories[i].begin() + readed_index + NUM_WAYPOINTS + 1,
                      trajectory2d.begin());
        }
        else
        {
            std::copy(readed_trajectories[i].begin() + readed_index,
                      readed_trajectories[i].end(),
                      trajectory2d.begin());
            for (int j = readed_index + NUM_WAYPOINTS + 1 - readed_trajectories[i].size(); j < NUM_WAYPOINTS + 1; ++j)
                trajectory2d[j] = readed_trajectories[i].back();
        }

        printf("Read trajectory %d - %d\n", readed_index, readed_index + NUM_WAYPOINTS);
    }
    readed_index += NUM_WAYPOINTS;

    return true;
}

int main(int argc, char **argv)
{
    std::string file_name = "new_input.txt";

    if (argc > 1)
        file_name = argv[1];

    if (argc > 2)
        readed_index = std::atoi(argv[2]);

    //temp vars
    bool success, stepSuccess;
    unsigned int messageType, numAgents, numFrames, numReplyAgents, agentSize, replyAgentSize;
    float **reply = 0x0;

    //launch client
    Interface3D::Interface3DClient client;

    std::cout << "Client Connect!" << std::endl;

    success = client.initSocket("localhost","5557");
    if (success)
    {
        std::cout << "socket connect!" << std::endl;
    }
    else
    {
        std::cout << "socket Fail!" << std::endl;
    }

    //confirm connection
    success = client.connectConfirm();
    if (success)
    {
        std::cout << "server connect!" << std::endl;
    }
    else
    {
        std::cout << "server Fail!" << std::endl;
        return 1;
    }


    std::cout << "Prepare an agent frame" <<std::endl;

    readFile(file_name, numAgents, numFrames, agentSize);

    float **agentData = new float*[numAgents * numFrames];
    for (int agent = 0; agent < numAgents * numFrames; ++agent)
    {
        agentData[agent] = new float[agentSize];
    }

    vector<Trajectory2D> trajectories;
    while(getInput(trajectories))
    {
        std::cout << "agents:" << std::endl;
        for (int frame = 0; frame < numFrames; ++frame)
        {
            for (int agent = 0; agent < numAgents; ++agent)
            {
                Waypoint2D& waypoint = trajectories[agent][frame];
                agentData[frame * numAgents + agent][0] = waypoint.frame;
                agentData[frame * numAgents + agent][1] = waypoint.agent_id;
                agentData[frame * numAgents + agent][2] = waypoint.state;
                agentData[frame * numAgents + agent][3] = waypoint.x;
                agentData[frame * numAgents + agent][4] = waypoint.y;
                agentData[frame * numAgents + agent][5] = waypoint.vx;
                agentData[frame * numAgents + agent][6] = waypoint.vy;
                agentData[frame * numAgents + agent][7] = waypoint.pvx;
                agentData[frame * numAgents + agent][8] = waypoint.pvy;
                agentData[frame * numAgents + agent][9] = waypoint.orientation;
                agentData[frame * numAgents + agent][10] = waypoint.radius;
                agentData[frame * numAgents + agent][11] = waypoint.neighbors.size();
                for (int neighbor = 0; neighbor < waypoint.neighbors.size(); ++neighbor)
                    agentData[frame * numAgents + agent][12 + neighbor]  = waypoint.neighbors[neighbor];

                cout << waypoint;
            }
        }

        std::cout << "built agent frame. Agents: " << numAgents << ", agent size: "<< agentSize << ", frames: " << numFrames << std::endl;

        //send the frame over
        success = client.sendAgentData(numFrames, numAgents, agentSize, agentData);
        std::cout << "Sent the frame." << std::endl;

        //get a response
        success = client.receiveMessage(messageType);
        success = client.handleAgentReply(messageType, stepSuccess, numReplyAgents, replyAgentSize, reply);

        //print the message and the data
        if (stepSuccess)
        {
            std::cout << "STEP Success: " <<std::endl;
        }
        else
        {
            std::cout << "STEP FAIL: " <<std::endl;
            readed_index -= NUM_WAYPOINTS;
        }
        for (unsigned int ragent = 0; ragent < numReplyAgents; ++ragent)
        {
            for (unsigned int rdata = 0; rdata < replyAgentSize; ++rdata)
            {
                std::cout << reply[ragent][rdata] << " ";
            }
            std::cout << std::endl;
        }
    }

    // memore deallocation
    for (int agent = 0; agent < numAgents * numFrames; ++agent)
    {
        delete[] agentData[agent];
    }
    delete[] agentData;

}
