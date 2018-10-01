#pragma once
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON


#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <chrono>

using namespace msr::airlib;

#define ROLL_RATE 0.1f
#define PITCH_RATE 0.1f
#define YAW_RATE 0.1f
#define LIFT_RATE 1.0f


class Session
{
private:
	MultirotorRpcLibClient *client;

public:
	Session();
	~Session();
	Session(MultirotorRpcLibClient *client);
	void disconnect();
	void connect();
	bool isConnected();
};

