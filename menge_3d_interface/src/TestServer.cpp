#include <menge_3d_interface/3DInterfaceServer.h>
#include <menge_3d_interface/3DInterfaceMessageTypes.h>

#include <iostream>
int main() {

	//launch server
	std::cout << "START A SERVER ON 5556" <<std::endl;

	Interface3D::Interface3DServer server;
	unsigned int msg = 0, numAgents=0,numFrames=0,agentSize=0;
	bool test = false;
	float **testArray=0x0;
	server.initSocket("*", "5557");

	std::cout << "Started" << std::endl;

	//  Prepare our context and socket
  

	while (true) {
		test = server.receiveMessage(msg);
		if (test && msg == Interface3D::CONNECT){
			server.handleConnectMessage();
			std::cout << "New Connection!" <<std::endl;
		} else if (test && msg == Interface3D::AGENT_DATA){
			std::cout << "got agent data message. Handle it." << std::endl;
			server.handleAgentDataMessage(numAgents,numFrames,agentSize, testArray);

			//print them
			for (int agent=0; agent < numAgents; ++agent){
				std::cout << "    " << testArray[agent][0] << ", " << testArray[agent][1] << ", " << testArray[agent][2] << " " << testArray[agent][3] << " " << testArray[agent][4] << std::endl;
			}

			//DECIDE PASS FAIL
			int pass = random() % 2;
			if (pass == 1){
				//build a response based on telling the client this frame is fine. The response is built by create an array of size 5
				float **response = new float*[numAgents];
				for (unsigned int agent = 0; agent < numAgents; ++agent){
					response[agent] = new float[5];
					response[agent][0] = testArray[numFrames-1 * numAgents + agent][0];
					response[agent][1] = testArray[numFrames-1 * numAgents + agent][1];
					response[agent][2] = testArray[numFrames-1 * numAgents + agent][2];
					response[agent][3] = testArray[numFrames-1 * numAgents + agent][3];
					response[agent][4] = 0.0;
				}

				//tell the client he succeeded
				server.sendSuccessMessage(numAgents, 5, response);
			} else {
				//TODO: This is a memory leak
				//build a response based on telling the client this frame is bad. The response is built by create an array of size 1
				float **response = new float*[numAgents];
				for (unsigned int agent = 0; agent < numAgents; ++agent){
					response[agent] = new float[1];
					response[agent][0] = testArray[agent][0];
				}

				//tell the client he failed
				server.sendFailMessage(numAgents, 1, response);
			}
		}
        sleep(1);
	}

}
