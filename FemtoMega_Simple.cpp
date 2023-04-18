#include <iostream>
#include <sstream>

#include "App.h"

int main(int argc, char** argv)
{
	try {
		App app;
		app.init();
		app.run();
	}
	catch (std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
		return -1;
	}
	return 0;
}