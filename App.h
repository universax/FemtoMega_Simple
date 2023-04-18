#pragma once
#include "FemtoMega.h"

class App
{
public:
	App() {}
	~App() {}

	void init();
	void run();

private:
	//cv::Mat update_depth();
	//cv::Mat update_color();
	//cv::Mat update_body();
	//cv::Mat update_pointcloud();

	// Femto mega
	FemtoMega fm;

	// Event
	bool _running;
	void handleKey(char key);

};
