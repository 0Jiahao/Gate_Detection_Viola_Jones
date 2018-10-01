#include "AirSimInterface.h"

AirSimInterface::AirSimInterface()
{
    client = new msr::airlib::MultirotorRpcLibClient;
    this->session = new Session(client);
}

AirSimInterface::~AirSimInterface()
{
	this->session->disconnect();
}

void AirSimInterface::setPose(float x, float y, float z,float phi,float theta,float psi) {
	Eigen::Quaternionf q;
	q = Eigen::AngleAxisf(phi, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(psi, Eigen::Vector3f::UnitZ());

	client->simSetPose(msr::airlib::Pose(Eigen::Vector3f({ x,y,z }), q),false);
}


void AirSimInterface::connect() {
	this->session->connect();
}

Image AirSimInterface::getImage()
{
    vector<ImageCaptureBase::ImageRequest> request = { ImageCaptureBase::ImageRequest(1, ImageCaptureBase::ImageType::Scene, false, false) };

    const vector<ImageCaptureBase::ImageResponse>& response = client->simGetImages(request);
//TODO check if there are multiple responses
    auto r = response[0];
    return Image(r.image_data_uint8,r.width,r.height);
}

bool AirSimInterface::isConnected() {
	return this->session->isConnected();
}

void AirSimInterface::autopilotControl() {
    client->enableApiControl(true);
}









/*

std::cout << "Press Enter to takeover API control" << std::endl; std::cin.get();
client.enableApiControl(true);
client.armDisarm(true);

float takeoffTimeout = 5;
client.takeoff(takeoffTimeout);

// switch to explicit hover mode so that this is the fallback when
// move* commands are finished.
std::this_thread::sleep_for(std::chrono::duration<double>(5));
client.hover();

bool running = TRUE;
const float speed = 3.0f;
const float size = 10.0f;
const float duration = size / speed;
//DrivetrainType driveTrain = DrivetrainType::ForwardOnly;
//		YawMode yaw_mode(true, 0);

MultirotorState state = client.getMultirotorState();
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float altitude = state.kinematics_true.pose.position.z();
std::cout << "Ready for keyboard control" << std::endl;

do {

client.moveByAngleZ(pitch, roll, altitude, yaw, duration);
roll = 0.0;
pitch = 0.0;

switch (std::cin.get()) {
case 'a':
roll += ROLL_RATE;
std::cout << "|Roll: " << roll << "||Pitch: " << pitch << "||Yaw: " << yaw << "||Height: " << altitude << "||" << std::endl;
break;
case 'd':
roll += ROLL_RATE;
std::cout << "|Roll: " << roll << "||Pitch: " << pitch << "||Yaw: " << yaw << "||Height: " << altitude << "||" << std::endl;

break;
case 'w':
pitch += PITCH_RATE;
std::cout << "|Roll: " << roll << "||Pitch: " << pitch << "||Yaw: " << yaw << "||Height: " << altitude << "||" << std::endl;

break;
case 's':
pitch -= PITCH_RATE;
std::cout << "|Roll: " << roll << "||Pitch: " << pitch << "||Yaw: " << yaw << "||Height: " << altitude << "||" << std::endl;
break;
case 'q':
yaw += YAW_RATE;
std::cout << "|Roll: " << roll << "||Pitch: " << pitch << "||Yaw: " << yaw << "||Height: " << altitude << "||" << std::endl;
break;
case 'e':
yaw -= YAW_RATE;
std::cout << "|Roll: " << roll << "||Pitch: " << pitch << "||Yaw: " << yaw << "||Height: " << altitude << "||" << std::endl;
break;
case '+':
altitude += LIFT_RATE;
std::cout << "|Roll: " << roll << "||Pitch: " << pitch << "||Yaw: " << yaw << "||Height: " << altitude << "||" << std::endl;
break;
case '-':
altitude -= LIFT_RATE;
std::cout << "|Roll: " << roll << "||Pitch: " << pitch << "||Yaw: " << yaw << "||Height: " << altitude << "||" << std::endl;
break;

case 'c':

std::cout << "Taking FPV Image" << std::endl; std::cin.get();
vector<ImageRequest> request = { ImageRequest(0, ImageType::Scene), ImageRequest(1, ImageType::DepthPlanner, true) };
const vector<ImageResponse>& response = client.simGetImages(request);
std::cout << "# of images recieved: " << response.size() << std::endl;

if (response.size() > 0) {
std::cout << "Enter path with ending separator to save images (leave empty for no save)" << std::endl;
std::string path;
std::getline(std::cin, path);

for (const ImageResponse& image_info : response) {
std::cout << "Image uint8 size: " << image_info.image_data_uint8.size() << std::endl;
std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;

if (path != "") {
std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
if (image_info.pixels_as_float) {
Utils::writePfmFile(image_info.image_data_float.data(), image_info.width, image_info.height,
file_path + ".pfm");
}
else {
std::ofstream file(file_path + ".png", std::ios::binary);
file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
file.close();
}
}
}
}
break;
			case 'h':
				client.hover();
				std::cout << "Hovering.." << std::endl;
				break;
			case 'x':
				std::cout << "Landing.." << std::endl;
				client.land();
				client.armDisarm(false);
				running = FALSE;
				break;
			}
			std::this_thread::sleep_for(std::chrono::duration<double>(duration));

		} while (running);

*/