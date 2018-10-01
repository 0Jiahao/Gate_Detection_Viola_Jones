/**
 * Dynamic Model.
 * Thread safe wrapper that contains sensor + drone model.
 */
#ifndef DRONE_MODEL_CONTAINER_H
#define DRONE_MODEL_CONTAINER_H
#include "DroneModel.h"
#include "SensorModel.h"
#include "Kinematics.h"
#include "MotionRead.h"
#include <thread>
#include <mutex>

namespace mav{


class DynamicModel {
protected:
	DroneModel model;
	SensorModel sensorModel;
	std::mutex mutex;
	Kinematics kin;
	MotionRead sensorReadings;
public:
	DynamicModel()=default;
	DynamicModel(DynamicModel &old);

	explicit DynamicModel(DroneModel &model, SensorModel &sensorModel);
	Kinematics update(float roll, float pitch, float yaw, float z);
	Kinematics getTrueKinematics();
	MotionRead getSensorReadings();

};
}

#endif