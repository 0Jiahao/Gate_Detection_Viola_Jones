#include <iostream>
#include <Comm.h>
#include <flightplan.h>
#include "PoseEstimator.h"



using namespace std;
namespace mav{


PoseEstimator::PoseEstimator()
{
	this->pos = {0,0,0};
	this->intrinsicParameter.c_x = CAMERA_C_X;
	this->intrinsicParameter.c_y = CAMERA_C_Y;
	this->intrinsicParameter.f = CAMERA_F;
	this->gate_e.p1 = Vector3d(0.0,-GATE_LENGTH/2.0,GATE_HEIGHT-GATE_LENGTH/2.0);
	this->gate_e.p2 = Vector3d(0.0,GATE_LENGTH/2.0,GATE_HEIGHT-GATE_LENGTH/2.0);
	this->gate_e.p3 = Vector3d(0.0,-GATE_LENGTH/2.0,GATE_HEIGHT+GATE_LENGTH/2.0);
	this->gate_e.p4 = Vector3d(0.0,GATE_LENGTH/2.0,GATE_HEIGHT+GATE_LENGTH/2.0);
	setEulerAngle();
	poseCounter = 0;
}
void PoseEstimator::estimatePose()
{


	std::ostringstream msg;
	msg << "[pose estimator] best gate p1 = (" << bestGate.get_vertex(1).col << "," << bestGate.get_vertex(1).row<<
		")" << std::endl;
	Comm::print(msg.str());
	// update AHRS reading
    setEulerAngle();
    // c -- camera frame e -- earth frame
	Vector3d v1_c(3,1),v2_c(3,1),v3_c(3,1),v4_c(3,1);
	setEulerAngle();
	Vector3d v1_e(3,1),v2_e(3,1),v3_e(3,1),v4_e(3,1);
	v1_c << 1,(bestGate.get_vertex(1).col-intrinsicParameter.c_x)/intrinsicParameter.f,
			(bestGate.get_vertex(1).row-intrinsicParameter.c_y)/intrinsicParameter.f;
	v2_c << 1,(bestGate.get_vertex(2).col-intrinsicParameter.c_x)/intrinsicParameter.f,
			(bestGate.get_vertex(2).row-intrinsicParameter.c_y)/intrinsicParameter.f;
	v3_c << 1,(bestGate.get_vertex(3).col-intrinsicParameter.c_x)/intrinsicParameter.f,
			(bestGate.get_vertex(3).row-intrinsicParameter.c_y)/intrinsicParameter.f;
	v4_c << 1,(bestGate.get_vertex(4).col-intrinsicParameter.c_x)/intrinsicParameter.f,
			(bestGate.get_vertex(4).row-intrinsicParameter.c_y)/intrinsicParameter.f;

//	std::cout << "[pose estimation] best gate = " << bestGate.get_vertex(1).col << ","<< bestGate.get_vertex(1).row  << std::endl;
//	std::cout << "[pose estimation] best gate = " << bestGate.get_vertex(2).col << ","<< bestGate.get_vertex(2).row  << std::endl;
//	std::cout << "[pose estimation] best gate = " << bestGate.get_vertex(3).col << ","<< bestGate.get_vertex(3).row  << std::endl;
//	std::cout << "[pose estimation] best gate = " << bestGate.get_vertex(4).col << ","<< bestGate.get_vertex(4).row  << std::endl<<std::endl;
//
//	std::cout << "[pose estimation] v1_c = " << v1_c(0) << ","<< v1_c(1) << ","<< v1_c(2)  << std::endl;
//	std::cout << "[pose estimation] v2_c = " << v2_c(0) << ","<< v2_c(1) << ","<< v2_c(2)  << std::endl;
//	std::cout << "[pose estimation] v3_c = " << v3_c(0) << ","<< v3_c(1) << ","<< v3_c(2)  << std::endl;
//	std::cout << "[pose estimation] v4_c = " << v4_c(0) << ","<< v4_c(1) << ","<< v4_c(2)  << std::endl<<std::endl;

    //std::ostringstream debugMsg;
    //debugMsg << "R_B_E = "<< R_B_E<< endl;
    //Comm::print(debugMsg.str());

	unitVector(v1_c);
	unitVector(v2_c);
	unitVector(v3_c);
	unitVector(v4_c);

    v_e.v1_e = transformCoordinate(v1_c,R_B_E);
	v_e.v2_e = transformCoordinate(v2_c,R_B_E);
	v_e.v3_e = transformCoordinate(v3_c,R_B_E);
	v_e.v4_e = transformCoordinate(v4_c,R_B_E);

	// http://cal.cs.illinois.edu/~johannes/research/LS_line_intersect.pdf
    Matrix3d R = Matrix3d::Zero();
	Vector3d q = Vector3d::Zero();

    //std::ostringstream debugMsg1;
    //debugMsg1 << "R = " << R << endl;
    //Comm::print(debugMsg1.str());

	get_R_and_q_ForLeastSquare(R,q);
	this->pos = R.colPivHouseholderQr().solve(q); // \hat{t} = R^{-1}q

	float psi = ahrsReadings.getPsi() - dr_fp.gate_psi;

	float deltaX = this->pos(0);
	float deltaY = this->pos(1);

	float rotatedX = cos(psi)*deltaX - sin(psi)*deltaY;
	float rotatedY = sin(psi)*deltaX + cos(psi)*deltaY;

	pos(0) = rotatedX;
	pos(1) = rotatedY;


	poseCounter++;
	//std::cout <<" [pose estimator : ] pose counter is "<< poseCounter << std::endl;
    //std::ostringstream debugMsg2;
    //debugMsg2 << "estimated position is   " << this->pos << endl;
    //Comm::print(debugMsg2.str());
	// print result to image



}

void PoseEstimator::setEulerAngle()
{

    guard.lock();
	double phi = ahrsReadings.getPhi();
	double theta = ahrsReadings.getTheta();
	//double psi = dr_fp.gate_psi-  ahrsReadings.getPsi();   // ahrsReadings.getPsi(); pose estimation is local
	//double psi = 0 -  ahrsReadings.getPsi();   // ahrsReadings.getPsi(); pose estimation is local
	//psi = -psi;
	float psi = 0.0;
    guard.unlock();
//
	R_E_B(0,0) = cos(theta)*cos(psi) ;
	R_E_B(0,1) = cos(theta)*sin(psi) ;
	R_E_B(0,2) = -sin(theta);
	R_E_B(1,0) = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
	R_E_B(1,1) = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
	R_E_B(1,2) = sin(phi)*cos(theta);
	R_E_B(2,0) = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
	R_E_B(2,1) = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
	R_E_B(2,2) = cos(phi)*cos(theta);
	R_B_E = R_E_B.transpose();

    //std::ostringstream debugMsg;
    //debugMsg << "[Pose::setEulerrange] R_E_B = " << R_E_B << endl << "[Pose::setEulerrange] R_B_E = " << R_B_E << endl;
    //Comm::print(debugMsg.str());
}

void PoseEstimator::get_R_and_q_ForLeastSquare(Matrix3d &R,Vector3d &q)
{
	Matrix3d I;
	I << 1,0,0,0,1,0,0,0,1;

    // R = \sum_{i=1}^{4}(I-n_in_i^{T})^{T}
    Matrix3d temp = (I-v_e.v1_e*v_e.v1_e.transpose());
	R = R + temp.transpose();
	temp = (I-v_e.v2_e*v_e.v2_e.transpose());
	R = R + temp.transpose();
	temp = (I-v_e.v3_e*v_e.v3_e.transpose());
	R = R + temp.transpose();
	temp = (I-v_e.v4_e*v_e.v4_e.transpose());
	R = R + temp.transpose();

    // q = \sum_{i=1}^{4}(I-n_in_i^{T})a_i
	q = q + (I - v_e.v1_e*v_e.v1_e.transpose())*gate_e.p1;
	q = q + (I - v_e.v2_e*v_e.v2_e.transpose())*gate_e.p2;
	q = q + (I - v_e.v3_e*v_e.v3_e.transpose())*gate_e.p3;
	q = q + (I - v_e.v4_e*v_e.v4_e.transpose())*gate_e.p4;
}


void PoseEstimator::unitVector(Vector3d & vec)
{
	double norm = sqrt(pow(vec(0),2)+pow(vec(1),2)+pow(vec(2),2));
	vec(0) = vec(0)/norm;
	vec(1) = vec(1)/norm;
	vec(2) = vec(2)/norm;

}
void PoseEstimator::updateAHRS(MotionRead ahrsReadings)
{
    guard.lock();
    this->ahrsReadings = ahrsReadings;
    guard.unlock();
}

Pose PoseEstimator::getCurrentPoseEstimate() {
    guard.lock();
    Pose pose(this->pos);
    guard.unlock();
    return pose;
}

void PoseEstimator::updateGate(Polygon gate) {
    guard.lock();
    this->bestGate = gate;
    guard.unlock();
}

}