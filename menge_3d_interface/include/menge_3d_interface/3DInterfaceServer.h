#ifndef INTERFACE_3D_SERVER
#define INTERFACE_3D_SERVER

//
//  Menge 3d Interface Server in c++
//  Binds REP socket to tcp://*:5555
//  Client tells the server the size of the vector to be sent. 
//  Client sends a vector. Server squares each element and returns it.
//


#include <sstream>
#include <string>
#include "zhelpers.hpp"
#include "3DInterfaceComponent.h"
#ifndef _WIN32
#include <unistd.h>

#else
#include <windows.h>

#define sleep(n)	Sleep(n)
#endif

//TODO: THIS IS STRICTLY BACK AND FORTH. ANYTHING ELSE WILL CRASH.

namespace Interface3D {

	class Interface3DServer : public Interface3DComponent {

	public:

		/*
			Default constructor
		*/
		Interface3DServer();


		/*
			Function to handle a client connect message 

			Returns: true / false on method success

			success means the client is now authed
			
		*/
		bool handleConnectMessage();

		
		/*
			Function to handle an agent data message. 

			Returns: true / false on method success, whether a new array could 

			Params: unsigned int to store the number of frames
					unsigned int to store the number of agents
			        unsigned int to store the size of each agent
					float **     to store the resulting agent data
		
			Header Information: id, message_type, num frames, num_agents, sizeof agent
			Row Information: 1 Row Per Agent Per Frame
			Col Information: frame# agent_id agent_radius pos_x pos_y orientation state pref_x pref_y vel_x vel_y num_neighbors [list of up to max_neighbors]
		*/

		bool handleAgentDataMessage(unsigned int &numFrames, unsigned int  &numAgents, unsigned int &agentSize, float **&agentData);

		/*
			Function to send a success message. 

			Returns: true / false on method success, whether a message was sent

			Params: unsigned int for the number of agents to be sent (num rows)
			        unsigned int to send the size of each agent. This Should be 5 always
					float **     the actual agent data

			Header Information: id, message_type, num_agents, sizeof agent
			Row Information: 1 Row Per Agent
			Col Information: agent_id pos_x pos_y orientation locked

			Locked means the agent is animating in High-DOF and needs to be kept still in Menge

		*/
		bool sendSuccessMessage(unsigned int numAgents, unsigned int agentSize, float **sendData);

		/*
			Function to send a failure message. 

			Returns: true / false on method success, whether a message was sent

			Params: unsigned int for the number of agents to be sent (num rows)
			        unsigned int to send the size of each agent. This Should be 1 always
					float **     the actual agent data

			Header Information: id, message_type, num_agents, sizeof agent
			Row Information: 1 Row Per Agent
			Col Information: agent_id

		*/
		bool sendFailMessage(unsigned int numAgents, unsigned int agentSize, float **sendData);

	};

};

#endif

