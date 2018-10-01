#include "DynamicModel.h"
namespace mav{
DynamicModel::DynamicModel(DroneModel &model, SensorModel &sensorModel){
	this->model = model;
	this->sensorModel = sensorModel;
    kin = {
            0,
            0,
            0,
            0,
            0,
            0,
            0,
           0,
            0,
            0,
            0,
            0,
            0,
            0,
           0
    };
	}

DynamicModel::DynamicModel(DynamicModel &old){
		model = old.model;
		kin = old.kin;
		}

Kinematics DynamicModel::update(float roll, float pitch, float yaw, float thrust) {
	mutex.lock();
	model.setCommand(roll, pitch, yaw, thrust);
	model.propagateDroneModel();
	kin = {
			model.getX(),
			model.getY(),
			model.getZ(),
			model.getPhi(),
			model.getTheta(),
			model.getPsi(),
			model.getP(),
			model.getQ(),
			model.getR(),
			model.getAccX(),
			model.getAccY(),
			model.getAccZ(),
			model.getVx(),
			model.getVy(),
			model.getVz()
	};
	sensorModel.update(kin);
	//sensorModel.complementaryFilterAHRS_run();
	sensorModel.kalmanFilterAHRS_run();
	sensorReadings = MotionRead(sensorModel.getPhiAHRS(), sensorModel.getThetaAHRS(), sensorModel.getPSiAHRS(),
		sensorModel.getAccX(), sensorModel.getAccY(), sensorModel.getAccZ(), sensorModel.getP(), sensorModel.getQ(), sensorModel.getR(),model.getZ());
	mutex.unlock();
	return kin;
}

	Kinematics DynamicModel::getTrueKinematics() {
        mutex.lock();
        auto kinRet = kin;
        mutex.unlock();
        return kinRet;
	}

	MotionRead DynamicModel::getSensorReadings()
	{
		mutex.lock();
		MotionRead ahrs = sensorReadings;
		mutex.unlock();
		return ahrs;
	}


}