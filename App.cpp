#include "App.h"
#include "NetworkUtilLibrary.h"

// std
#include <thread>
#include <mutex>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// CV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>


void App::init() {
	//// Check all devices on the network
	//try {
	//	std::vector<std::pair<std::string, std::string>> device_info = NetworkUtilLibrary::getInstance().listDeviceIDsAndIPsOnLAN();
	//	std::cout << "Found " << device_info.size() << " devices on the network." << std::endl;
	//	for (const auto& i : device_info) {
	//		std::cout << "Found device ID: " << i.first << " with IP address: " << i.second << std::endl;
	//	}
	//}
	//catch (const std::runtime_error& e) {
	//	std::cerr << "Error: " << e.what() << std::endl;
	//}

	
}

void App::run() {
	// Create femto mega device conncection info
	std::vector <std::pair<std::string, int>> input_ip_address_and_ports;
	std::pair<std::string, int> ip_address_and_port_1("192.168.1.201", 8090);
	input_ip_address_and_ports.push_back(ip_address_and_port_1);
	std::pair<std::string, int> ip_address_and_port_2("192.168.1.202", 8090);
	input_ip_address_and_ports.push_back(ip_address_and_port_2);

	_running = fm.init(input_ip_address_and_ports);

	// start stream
	if(_running) fm.start();
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	fm.stop();

	return;
	// main loop
	while (_running)
	{
		// Calc framerate for this function
		cv::TickMeter tm;
		tm.start();
		// --------------------------------------------

		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		// --------------------------------------------
		// print framerate
		tm.stop();
		printf("FPS: %f \n", tm.getFPS());

		// wait for key
		handleKey((char)cv::waitKey(100));
	}

	
}

// key bind
void App::handleKey(char key)
{
	switch (key) {
	case 27:
		_running = false;
		break;
	default:
		break;
	}
}