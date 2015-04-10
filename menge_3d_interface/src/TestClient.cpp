#include <menge_3d_interface/3DInterfaceClient.h>
#include <menge_3d_interface/3DInterfaceMessageTypes.h>
#include <iostream>
#include <random>

int main() {

	//temp vars
	bool success, stepSuccess;
	unsigned int messageType, numAgents, numFrames, numReplyAgents, agentSize, replyAgentSize;
	float **reply = 0x0;
	
	//launch client
	Interface3D::Interface3DClient client;

	std::cout << "Client Connect!" << std::endl;
	
	success = client.initSocket("localhost","5557");
	if (success) {
		std::cout << "socket connect!" << std::endl;
	} else {
		std::cout << "socket Fail!" << std::endl;
	}
	
	//confirm connection
	success = client.connectConfirm();
	if (success) {
		std::cout << "server connect!" << std::endl;
	} else {
		std::cout << "server Fail!" << std::endl;
		return 1;
	}


	std::cout << "Prepare an agent frame" <<std::endl;

	numAgents = 5;
	numFrames = 5;
	agentSize = 5;

	//TODO: Memory leak
	float **agentData = new float*[numAgents * numFrames];
	for (int agent = 0; agent < numAgents * numFrames; ++agent){
		agentData[agent] = new float[agentSize];
	}
	
	//fake agent data
	agentData[0][0] = 0.0;
	agentData[1][0] = 1.0;
	agentData[2][0] = 2.0;
	agentData[3][0] = 3.0;
	agentData[4][0] = 4.0;

	//set fake positions, vprefs, and states
	for (int agent=0; agent < numAgents; ++agent){
		agentData[agent][1] = int(random() % 20 - 10.0);
		agentData[agent][2] = int(random() % 20 - 10.0);
		agentData[agent][3] = random() % 2 - 1.0;
		agentData[agent][4] = random() % 2 - 1.0;
	}

	//set the fake frames
	for (int frame=1; frame < numFrames; ++frame){
		for (int agent=0; agent < numAgents; ++agent){
			agentData[frame * numAgents + agent][3] = agentData[(frame-1) * numAgents + agent][3];
			agentData[frame * numAgents + agent][4] = agentData[(frame-1) * numAgents + agent][4];
			agentData[frame * numAgents + agent][1] = agentData[(frame-1) * numAgents + agent][1];
			agentData[frame * numAgents + agent][2] = agentData[(frame-1) * numAgents + agent][2];
			

			agentData[frame * numAgents + agent][1] += agentData[frame * numAgents + agent][3];
			agentData[frame * numAgents + agent][2] += agentData[frame * numAgents + agent][4];
		}
	}
	std::cout << "agents:" << std::endl;
	for (int agent=0; agent < numAgents; ++agent){
		std::cout << "    " << agentData[agent][0] << ", " << agentData[agent][1] << ", " << agentData[agent][2] << ", " << agentData[agent][3] << ", " << agentData[agent][4] << std::endl;
	}

	std::cout << "built agent frame. Agents: " << numAgents << ", agent size: "<< agentSize << ", frames: " << numFrames << std::endl;
	
	
	//send the frame over
	success = client.sendAgentData(numFrames, numAgents, 5, agentData);
	std::cout << "Sent the frame." << std::endl;

	//get a response
	success = client.receiveMessage(messageType);
	success = client.handleAgentReply(messageType, stepSuccess, numReplyAgents, replyAgentSize, reply);

	//print the message and the data
	if (stepSuccess) {
		std::cout << "STEP Success: " <<std::endl;
	} else {
		std::cout << "STEP FAIL: " <<std::endl;
	}
	for (unsigned int ragent = 0; ragent < numReplyAgents; ++ragent) {
		for (unsigned int rdata = 0; rdata < replyAgentSize; ++rdata) {
			std::cout << reply[ragent][rdata] << " ";
		}
		std::cout << std::endl;
	}
		
}
