
#include <menge_3d_interface/3DInterfaceMessageTypes.h>
#include <menge_3d_interface/3DInterfaceServer.h>

namespace Interface3D {


	///////////////////////////////////////
	//////// Interface3dServer
	///////////////////////////////////////

	Interface3DServer::Interface3DServer(): Interface3DComponent(true) {}
		
	///////////////////////////////////////////////////////////////////////////////////////

	bool Interface3DServer::handleConnectMessage() {
		
		//There is no more to this message. Just send back acknowledgement
		_outStreamer.clear();
		_outStreamer.str("");
		_outStreamer << InterfaceMessageType::ACCEPT_CONNECT << " " << _lastID;
		s_send(*_socket,_outStreamer.str());
		return true;
	}

	////////////////////////////////////////////////////////////////////////////////////
	bool Interface3DServer::handleAgentDataMessage(unsigned int &numFrames, unsigned int  &numAgents, unsigned int &agentSize, float **&agentData){
	
		//get the header
		_inStreamer >> numFrames >> numAgents >> agentSize;
		
		sizeArray(numFrames, numAgents, agentSize, agentData);

		//load the data for each agent in this data set
		for (int frame = 0; frame < numFrames; ++frame){
			for (int agent = 0; agent < numAgents; ++agent){
				for (int data = 0; data < agentSize; ++data){
					_inStreamer >> agentData[frame  *numAgents + agent][data];
				}
			}
		}

		std::cout << "    Agent Data Loaded."<<std::endl;
		return true;	
	}

	////////////////////////////////////////////////////////////////////////////////
	
	bool Interface3DServer::sendSuccessMessage(unsigned int numAgents, unsigned int agentSize, float **sendData){
		
		//prepare a message
		std::cout << "sending agent success to client." << std::endl;
		//sizeArray(numFrames,agentData);

		_outStreamer.clear();
		_outStreamer.str("");
		std::cout << "    packing message. " <<std::endl;
		_outStreamer << InterfaceMessageType::STEP_SUCCESS << " " << _lastID;
		_outStreamer << " " << numAgents << " " << agentSize;  
		for (int agent = 0; agent < numAgents; ++agent){
			for (int data = 0; data < agentSize; ++data){
				_outStreamer << " " << sendData[agent][data];
			}
		}

		std::cout << "    sending message. " <<std::endl;
		s_send(*_socket,_outStreamer.str());
		return true;
	
	}
	
	///////////////////////////////////////////////////////////////////////////////////

	bool Interface3DServer::sendFailMessage(unsigned int numAgents, unsigned int agentSize, float **sendData) {
		
		//prepare a message
		std::cout << "sending agent failure to client." << std::endl;
		//sizeArray(numFrames,agentData);

		_outStreamer.clear();
		_outStreamer.str("");
		std::cout << "    packing message. " <<std::endl;
		_outStreamer << InterfaceMessageType::STEP_FAIL << " " << _lastID;
		_outStreamer << " " << numAgents << " " << agentSize;  
		for (int agent = 0; agent < numAgents; ++agent){
			for (int data = 0; data < agentSize; ++data){
				_outStreamer << " " << sendData[agent][data];
			}
		}

		std::cout << "    sending fail message. " <<std::endl;
		s_send(*_socket,_outStreamer.str());
		return true;
	
	};
	
};

