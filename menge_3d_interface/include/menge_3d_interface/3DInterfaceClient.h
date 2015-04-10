#ifndef INTERFACE_3D_CLIENT
#define INTERFACE_3D_CLIENT

//
//  Vector Server in c++
//  Binds REP socket to tcp://*:5555
//  Client tells the server the size of the vector to be sent. 
//  Client sends a vector. Server squares each element and returns it.
//


#include <string>
#include <sstream>
#include <vector>
#include "3DInterfaceComponent.h"
#include "zhelpers.hpp"
#ifndef _WIN32
#include <unistd.h>

#else
#include <windows.h>

#define sleep(n)	Sleep(n)
#endif


namespace Interface3D {

	class Interface3DClient : public Interface3DComponent  {

	public:
		/*
			default constructor

		*/
		Interface3DClient();

		/*
			Function to send a connect handshake

			Returns: true / false on method success, whether the server accepted and responded

		*/
		bool connectConfirm();

		/*
			Function to send an agent data message

			Returns: true / false on method success, whether a message was sent

			Params: unsigned int the number of frames
					unsigned int the number of agents
			        unsigned int the size of each agent
					float **     agent data to send
		
			Header Information: id, message_type, num frames, num_agents, sizeof agent
			Row Information: 1 Row Per Agent Per Frame
			Col Information: frame# agent_id agent_radius pos_x pos_y orientation state pref_x pref_y vel_x vel_y num_neighbors [list of up to max_neighbors]

		*/
		bool sendAgentData(unsigned int numFrames, unsigned int numAgents, unsigned int agentSize, float** agentData);
		
		/*
			Function to handle the reply to an agent message

			Returns: true / false on method success, whether a message was received

			Params: unsigned int for message type
			        bool for success or fail (which the server sent)
					unsigned int to store the number of agents
			        unsigned int to store the size of each agent
					float **     agent data to send
		
			Header Information: id, message_type, num frames, num_agents, sizeof agent
			Row Information: 1 Row Per Agent Per Frame
			Col Information: either

			    agent_id pos_x pos_y orientation locked

			    Locked means the agent is animating in High-DOF and needs to be kept still in Menge

			or:

				agent_id

		*/

		bool handleAgentReply(unsigned int msg_type, bool &success, unsigned int &numAgents, unsigned int &agentSize, float** &agentData);
		
	};

};

#endif
