#pragma once
#include <string>
#include "Frame.h"
#include "Pose.h"
#include "SnakeGateDetector.h"
#include <Eigen/Dense>
#include <MotionRead.h>
#include <mutex>
#include "Config.h"




using namespace Eigen;
namespace mav{


struct IntrinsicParameter{
	double f;
	double c_x;
	double c_y;
};


struct gateCornerCoordinateEarth{
	Vector3d p1;
	Vector3d p2;
	Vector3d p3;
	Vector3d p4;
};

struct gateBearingVectorEarth{
	Vector3d v1_e;
	Vector3d v2_e;
	Vector3d v3_e;
	Vector3d v4_e;
};

class PoseEstimator
{
public:
	~PoseEstimator() = default;
    PoseEstimator(const PoseEstimator &that) = default;
    PoseEstimator();
	void updateGate(Polygon gate);
	void estimatePose();
    void setEulerAngle();
    void updateAHRS(MotionRead ahrsReadings);
    Pose getCurrentPoseEstimate();
	void setFocalLength(float f){
		intrinsicParameter.f = f;
	}

	void setPrincipalX(float cx){
		intrinsicParameter.c_x = cx;
	}

	void setPrincipalY(float cy){
		intrinsicParameter.c_y = cy;
	}
private:
	std::mutex guard;
	Vector3d pos;
	MotionRead ahrsReadings;
	struct IntrinsicParameter intrinsicParameter;
	struct gateCornerCoordinateEarth gate_e;
	struct gateBearingVectorEarth v_e;
	Polygon bestGate;
	Matrix3d R_B_E;
	Matrix3d R_E_B;
	int poseCounter;

	Vector3d transformCoordinate(Vector3d & vec,Matrix3d & R){return R*vec;};
	void unitVector(Vector3d & vec);
	void get_R_and_q_ForLeastSquare(Matrix3d &R,Vector3d &q);


};

}