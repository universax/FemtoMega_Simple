#pragma once
#include "Singleton.h"

#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <stdexcept>

#include <Winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "Ws2_32.lib")

class NetworkUtilLibrary: public Singleton<NetworkUtilLibrary>
{
public:
	// Returns a list of device IDs on the local area network
	std::vector<std::pair<std::string, std::string>> listDeviceIDsAndIPsOnLAN();

	// Returns true if the input string is a valid IP address and port number
	bool isValidIPAddressAndPort(const std::string& input, int port);

private:
	friend class Singleton<NetworkUtilLibrary>;
	NetworkUtilLibrary() = default;
	~NetworkUtilLibrary() {}

	void set_socket_timeout(int sockfd, int seconds);
};

