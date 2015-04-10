#ifndef INTERFACE_3D_MESSAGES
#define INTERFACE_3D_MESSAGES
namespace Interface3D {

	enum InterfaceMessageType {
		NO_MESSAGE,
		CONNECT,
		ACCEPT_CONNECT,
		AGENT_DATA,
		STEP_SUCCESS,
		STEP_FAIL
	};
}

#endif