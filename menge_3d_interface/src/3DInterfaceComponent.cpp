//  Menge 3d Interface Component in c++
//  Provides common functions to the 3D interface client and server


#include <menge_3d_interface/zhelpers.hpp>
#include <iostream>
#include <vector>
#include <menge_3d_interface/3DInterfaceMessageTypes.h>
#include <menge_3d_interface/3DInterfaceComponent.h>

//TODO: THIS IS STRICTLY BACK AND FORTH. ANYTHING ELSE WILL CRASH. 100% requires a reply message for EVERY MESSAGE.

namespace Interface3D {

	///////////////////////////////////////////////// Interface3DComponent //////////////////////////////////////////


	Interface3DComponent::Interface3DComponent(bool server) : _port(""),
		_ip(""), _outStreamer(), _responseString(), _inStreamer(), _server(server), _lastID(0)  {
		_context = new zmq::context_t(1);
		if (_server)
			_socket = new zmq::socket_t(*_context,ZMQ_REP);
		else
			_socket = new zmq::socket_t(*_context,ZMQ_REQ);
		_dataPool = 0x0;
		_lastData = 0x0;

	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
	bool Interface3DComponent::initSocket(std::string ip, std::string port) {
		_ip = ip;
		_port = port;

		if (_server) { 
			_port = port;
			_outStreamer.clear();
			_outStreamer.str("");
			_outStreamer << "tcp://" << _ip << ":" << _port;

			std::cout <<"bind to " << _outStreamer.str() << std::endl;
			_socket->bind (_outStreamer.str().c_str());
			return true;
		} else {

			//this is complicated. I need to know the server is there, and conect to it
			//this method will not "auth", the client needs to do that
			
			_outStreamer.clear();
			_outStreamer.str("");
			_outStreamer << "tcp://" << _ip << ":" << _port;
			std::cout << "Connecting to server at " << _outStreamer.str() << std::endl;
			_socket->connect (_outStreamer.str().c_str());

			return true;

		}
	};

	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////


	bool Interface3DComponent::receiveMessage(unsigned int &messageType) {
		///This method gets a message and loads ONLY its type. we decide what to do with it	
		
		//get the okay!
		if (_socket->connected()){
			
			_responseString = s_recv(*_socket);
			

			_inStreamer.clear();
			_inStreamer.str(_responseString);
			_inStreamer >> messageType;

			if (_server) {

				//get the id
				_inStreamer >> _lastID;
			} else {
				//make sure the id matches
				unsigned int testId;
				_inStreamer >> testId;

				assert(testId == _lastID++ && "Server returned a message out of order.");
				
			}

			return true;
		}

		return false;
	};


	//////////////////////////////////////////////////////////////////////////////////////////


	bool Interface3DComponent::sizeArray(unsigned int numFrames, unsigned int numAgents, unsigned int agentSize, float **&frame) {
		//clear my memory
		if (_dataPool != 0x0){
			delete _dataPool;
			_dataPool = 0x0;
		}

		if (_lastData != 0x0){
			delete(_lastData);
		}

		
		//allocate a pool of memory
		_lastData = new float*[numAgents * numFrames];
		_dataPool = new float[numAgents * agentSize * numFrames];
		float *pool = _dataPool;

		for (int i = 0; i < numAgents * numFrames; ++i, pool += agentSize){
			_lastData[i] = pool;
		}

		frame = _lastData;

		return true;
	};
};
