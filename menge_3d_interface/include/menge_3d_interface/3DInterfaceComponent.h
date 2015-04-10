#ifndef INTERFACE_3D_COMPONENT
#define INTERFACE_3D_COMPONENT

//
//  Menge 3d Interface Component in c++
//  Provides common functions to the 3D interface client and server


#include <string>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "zhelpers.hpp"
#ifndef _WIN32
#include <unistd.h>

#else
#include <windows.h>

#define sleep(n)	Sleep(n)
#endif

//TODO: THIS IS STRICTLY BACK AND FORTH. ANYTHING ELSE WILL CRASH. 100% requires a reply message for EVERY MESSAGE.

namespace Interface3D {

	class Interface3DComponent {
	public:
		/*
			constructor

			Params: boolean on whether I am server or client
		*/
		Interface3DComponent(bool server);

		/*
			Function to initiate the socket. 

			Returns: true / false on method success

			Params: string: ip to bind on / listen on
					string: port to connect on / listen on
		*/
		bool initSocket(std::string ip, std::string port);


		/*
			Function to receive a message from the other end.

			Returns: true / false on message received or not

			Params: unsigned integer to hold message type by reference
		*/
		bool receiveMessage(unsigned int &messageType);



	protected:

		
		/*
			Function to resize the internally managed memory to the interface
			No one should be calling this but me.

			Returns: true / false on array resize success

			Params: unsigned integer to for the number of frames I need to store & buffer
			        unsigned integer for number of agents
					unsigned integer for the size of the agents (per agent data size)
					2d floating point pointer by reference: the array I am going to use when creating data
		*/
		bool sizeArray(unsigned int numFrames, unsigned int numAgents, unsigned int agentSize, float **&frame);
		
		
		//socket data
		bool _server;
		std::string _ip;
		std::string _port;
		zmq::context_t *_context;
		zmq::socket_t *_socket;


		//keep these around for sending data accross
		std::ostringstream _outStreamer;
		std::string _responseString;
		std::istringstream _inStreamer;

		//last message id
		int _lastID;

		//pointer to last message array
		float ** _lastData;
		float *  _dataPool;
		
	};

};

#endif

