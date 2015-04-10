#include <menge_3d_interface/3DInterfaceClient.h>
#include <menge_3d_interface/3DInterfaceMessageTypes.h>

namespace Interface3D {

	//////////////////////////////////////////////////////////////
	///////////////////// Interface3DClient//////////////////////
	/////////////////////////////////////////////////////////////
	

	Interface3DClient::Interface3DClient() : Interface3DComponent(false) {};

	///////////////////////////////////////////////////////////////

	bool Interface3DClient::connectConfirm(){
	
		bool success;

		//this used to be a "connect message" function
		_outStreamer.clear();
		_outStreamer.str("");
		_outStreamer << InterfaceMessageType::CONNECT << " " << _lastID;
		s_send(*_socket,_outStreamer.str());
		
		///this used to be a receive connect reply message

		unsigned int test_id;
		//get the okay!
		success = receiveMessage(test_id);

		if (!success){
			std::cout <<"I did not understand the server connect reply." << std::endl;
		}

		success = (test_id == InterfaceMessageType::ACCEPT_CONNECT);
		
		if (!success){
			std::cout << "Server did not acknoweldege connection. " << std::endl;
			return false;
		}

		std::cout <<"Connected. I think..." <<std::endl;
		return true;
	};

	///////////////////////////////////////////////////////////////
	bool Interface3DClient::sendAgentData(unsigned int numFrames, unsigned int numAgents, unsigned int agentSize, float** agentData){
	

		std::cout << "sending agent data to server." << std::endl;
		//sizeArray(numFrames,agentData);

		_outStreamer.clear();
		_outStreamer.str("");
		std::cout << "    packing message. " <<std::endl;
		_outStreamer << InterfaceMessageType::AGENT_DATA << " " << _lastID;
		_outStreamer << " " << numFrames << " " << numAgents << " " << agentSize;
		for (int frame = 0; frame < numFrames; ++frame){
			for (int agent = 0; agent < numAgents; ++agent){
				for (int data = 0; data < agentSize; ++data){
					_outStreamer << " " << agentData[frame  *numAgents + agent][data];
				}
				//std::cout << "         agent " << agent << " done for frame " << frame << std::endl;
			}
		}
		std::cout << "    sending message. " <<std::endl;
		s_send(*_socket,_outStreamer.str());

		return true;
	};

	////////////////////////////////////////////////////////////////////////////////////

	bool Interface3DClient::handleAgentReply(unsigned int msg_type, bool &success, unsigned int &numAgents, unsigned int &agentSize, float** &agentData){
	
		//determine which message I got.
		if (msg_type == InterfaceMessageType::STEP_SUCCESS) {
			success = true;
			_inStreamer >> numAgents >> agentSize; 
		} else {
			success = false;
			_inStreamer >> numAgents >> agentSize; 
		}
		sizeArray(1, numAgents, agentSize, agentData);
		for (int agent = 0; agent < numAgents; ++agent){
			for (int data = 0; data < agentSize; ++data){
				_inStreamer >> agentData[agent][data];
			}
		}

		return true;
	};


};


