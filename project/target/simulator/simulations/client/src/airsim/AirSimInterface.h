/**
 * AirsimInterface
 * An interface to airsim.
 */
#pragma once
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#ifndef AIRSIM_INTERFACE_H
#define AIRSIM_INTERFACE_H
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>
#include <Image.h>
#include "Session.h"

using namespace msr::airlib;

class AirSimInterface
{
protected:
	Session *session;

	MultirotorRpcLibClient *client;
public:
	AirSimInterface();
	~AirSimInterface();

	void autopilotControl();
	void connect();
	/**
	 * Update camera pose
	 * @param x
	 * @param y
	 * @param z
	 * @param phi
	 * @param theta
	 * @param psi
	 */
	void setPose(float x, float y, float z,float phi, float theta, float psi);
	Image getImage();
	bool isConnected();

};

#endif