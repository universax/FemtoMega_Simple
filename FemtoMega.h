#pragma once
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

#include <thread>
#include <mutex>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <stdexcept>


class FemtoMega
{
public:
	FemtoMega() {}
	~FemtoMega() {}

	bool init(const std::vector<std::pair<std::string, int>>& input_ip_address_and_ports);
	void start();
	void stop();

private:
	std::vector<std::pair<std::string, int>> ip_address_and_ports;
	std::vector<std::shared_ptr<ob::Device>> devices;
	ob::Context ctx;
	std::vector<std::shared_ptr<ob::Pipeline>> pipelines;


	void startStream(std::vector<std::shared_ptr<ob::Device>> devices, OBSensorType sensorType, int deviceIndexBase);
	void stopStream(std::vector<std::shared_ptr<ob::Device>> devices, OBSensorType sensorType, int deviceIndexBase);

	std::mutex frameMutex;
	std::vector<std::shared_ptr<ob::Frame>> colorFrames;
	std::vector<std::shared_ptr<ob::Frame>> depthFrames;
	void handleColorStream(int devIndex, std::shared_ptr<ob::Frame> frame);
	void handleDepthStream(int devIndex, std::shared_ptr<ob::Frame> frame);
};

