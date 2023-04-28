#pragma once
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

// CV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

// PCL
#include "PCL_Functions.h"
#include "VisualizerManager.h"
#include "Sensor.h"

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

	// Getter
	std::vector<pcl::PointCloud<PointType>::Ptr> getPointclouds() {
		return pointclouds;
	}

private:
	// init
	bool initSensors(const std::vector<std::pair<std::string, int>>& ipAddressAndPorts);
	void initPointCloud();

	// update
	void updateFrameSet();
	void updateColorDepth();
	void updatePointCloud();

	// draw
	void drawColorMat();
	void drawDepthMat();

	// Util
	void showDeviceInfo(const std::shared_ptr<ob::Device> device);
		
	// Sensor
	ob::Context ctx;
	std::vector<std::shared_ptr<ob::Pipeline>> pipelines;

	// Frameset
	std::vector<std::shared_ptr<ob::FrameSet>> framesets;
	std::vector<std::shared_ptr<ob::ColorFrame>> colorFrames;
	std::vector<std::shared_ptr<ob::DepthFrame>> depthFrames;

	// Stream profile
	std::shared_ptr<ob::VideoStreamProfile> depthStreamProfile = nullptr;
	std::shared_ptr<ob::VideoStreamProfile> colorStreamProfile = nullptr;

	// CV
	std::vector<cv::Mat> colorMats, depthMats; 

	// Pointcloud
	OBFormat format = OBFormat::OB_FORMAT_POINT;
	std::vector<std::shared_ptr<ob::PointCloudFilter>> pointcloudFilters;

	// PCL
	std::vector<pcl::PointCloud<PointType>::Ptr> pointclouds;
};

