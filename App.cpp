#include "App.h"
#include "PCL_Functions.h"
#include "VisualizerManager.h"

void App::init() {
	// Create femto mega device conncection info
	std::vector <std::pair<std::string, int>> inputIpAddressAndPorts;
	std::pair<std::string, int> ip_address_and_port_1("192.168.1.201", 8090);
	inputIpAddressAndPorts.push_back(ip_address_and_port_1);
	std::pair<std::string, int> ip_address_and_port_2("192.168.1.202", 8090);
	inputIpAddressAndPorts.push_back(ip_address_and_port_2);

	_running = fm.init(inputIpAddressAndPorts);
}

void App::run() {
	// Visualizer
	VisualizerManager vm;

	// Registration
	std::vector<Eigen::Matrix4f> icpResults;

	// main loop
	while (_running)
	{
		// Calc framerate for this function
		cv::TickMeter tm;
		tm.start();
		// --------------------------------------------
		fm.update();

		{
			// Check running
			_running = vm.running;

			// Get pointclouds
			std::vector<pcl::PointCloud<PointType>::Ptr> pointclouds = fm.getPointclouds();
			if (pointclouds.empty()) continue;

			// Filter
			float defaultVoxelSize = 0.01f;
			for (size_t i = 0; i < pointclouds.size(); i++)
			{
				PCL_Functions::voxelGridFilter(defaultVoxelSize, pointclouds[i]);
			}

			// Pre-Transform


			// Registration
			if (pointclouds.size() > 1 && icpResults.empty())
			{
				pcl::PointCloud<PointType>::Ptr tgtPcd = pointclouds[0]; // this cloud is base
				// calc target pointcloud's transfrom
				for (size_t i = 0; i < pointclouds.size(); i++)
				{
					pcl::PointCloud<PointType>::Ptr srcPcd = pointclouds[i];
					Eigen::Matrix4f icpResult = PCL_Functions::iterativeClosestPoint(tgtPcd, srcPcd);
					icpResults.push_back(icpResult);
				}

				vm.registration = true;
			}

			// Transfrom
			if (icpResults.size() > 1)
			{
				pcl::PointCloud<PointType>::Ptr transformedPcd(new pcl::PointCloud<PointType>);
				for (size_t i = 1; i < pointclouds.size(); i++)
				{
					PCL_Functions::transform(pointclouds[i], icpResults[i]);
				}
			}

			//// Plane removal
			//for (size_t i = 0; i < pointclouds.size(); i++)
			//{
			//	PCL_Functions::planeRemoval(pointclouds[i], defaultVoxelSize * 5);
			//}


			// Marge and draw
			pcl::PointCloud<PointType>::Ptr mergedPcd(new pcl::PointCloud<PointType>);
			for (size_t i = 0; i < pointclouds.size(); i++)
			{
				PCL_Functions::voxelGridFilter(0.03f, pointclouds[i]);

				*mergedPcd += *pointclouds[i];
			}
			vm.updateVisualizer(mergedPcd);
		}

		// --------------------------------------------
		// print framerate
		tm.stop();
		//printf("FPS: %f \n", tm.getFPS());

		// wait for key
		handleKey((char)cv::waitKey(1));
	}
	fm.stop();
}

// key bind
void App::handleKey(char key)
{
	switch (key) {
	case 27:
		_running = false;
		break;
	case 'r':
		
		break;
	default:
		break;
	}
}