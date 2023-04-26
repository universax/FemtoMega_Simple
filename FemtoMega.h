#pragma once
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

// CV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"



class FemtoMega
{
public:
	FemtoMega() {}
	~FemtoMega() {
		stop();
	}

	bool init(const std::vector<std::pair<std::string, int>>& ipAddressAndPorts);
	void update();
	void draw();
	void stop();

	void process();
	void registration();

private:
	// init
	bool initSensors(const std::vector<std::pair<std::string, int>>& ipAddressAndPorts);
	void initPointCloud();

	// update
	void updateFrameSet();
	void updateColorDepth();
	void updatePointCloud();

	// draw
	void drawColorDepth();
	void drawPointcloud();

	// process
	void processPointCloud();
	void registrationPointcloud();

	// Util
	void showDeviceInfo(const std::shared_ptr<ob::Device> device);


	std::vector<std::pair<std::string, int>> ipAddressAndPorts;
	std::vector<std::shared_ptr<ob::Device>> devices;
	ob::Context ctx;
	std::vector<std::shared_ptr<ob::Pipeline>> pipelines;

	// Frameset
	std::vector<std::shared_ptr<ob::FrameSet>> framesets;

	// Stream profile
	std::shared_ptr<ob::VideoStreamProfile> depthStreamProfile = nullptr;
	std::shared_ptr<ob::VideoStreamProfile> colorStreamProfile = nullptr;

	std::mutex frameMutex;
	std::vector<std::shared_ptr<ob::ColorFrame>> colorFrames;
	std::vector<std::shared_ptr<ob::DepthFrame>> depthFrames;

	// CV
	std::vector<cv::Mat> colorMats, depthMats; 

	// Pointcloud
	OBFormat format = OBFormat::OB_FORMAT_POINT;
	std::vector<std::shared_ptr<ob::PointCloudFilter>> pointcloudFilters;
};

