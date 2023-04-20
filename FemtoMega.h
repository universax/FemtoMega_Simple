#pragma once
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

// std
#include <thread>
#include <mutex>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <stdexcept>

// Open3D
#include <open3d/Open3D.h>

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

private:
	// init
	bool initSensors(const std::vector<std::pair<std::string, int>>& ipAddressAndPorts);
	void initPointCloud();

	// update
	void updateColorDepth();
	void updatePointCloud();

	// draw
	void drawColorDepth();
	void drawPointcloud();

	// Util
	void showDeviceInfo(const std::shared_ptr<ob::Device> device);


	std::vector<std::pair<std::string, int>> ipAddressAndPorts;
	std::vector<std::shared_ptr<ob::Device>> devices;
	ob::Context ctx;
	std::vector<std::shared_ptr<ob::Pipeline>> pipelines;

	// Frameset
	std::shared_ptr<ob::FrameSet> frameset = nullptr;

	// Stream profile
	std::shared_ptr<ob::VideoStreamProfile> depthStreamProfile = nullptr;
	std::shared_ptr<ob::VideoStreamProfile> colorStreamProfile = nullptr;

	std::mutex frameMutex;
	std::vector<std::shared_ptr<ob::ColorFrame>> colorFrames;
	std::vector<std::shared_ptr<ob::DepthFrame>> depthFrames;

	// CV
	std::vector<cv::Mat> colorMats, depthMats; 

	// Open3D
	OBFormat format = OBFormat::OB_FORMAT_POINT;
	std::shared_ptr<ob::PointCloudFilter> pointcloudFilter;
	std::shared_ptr<ob::Frame> pointcloudFrame = nullptr;
	std::shared_ptr<open3d::geometry::PointCloud> pointcloud = nullptr;
	open3d::visualization::VisualizerWithKeyCallback visualizer;

};

