#include "NetworkUtilLibrary.h"

std::vector<std::pair<std::string, std::string>> NetworkUtilLibrary::listDeviceIDsAndIPsOnLAN() {
    SOCKET sockfd;
    sockaddr_in broadcast_addr;
    std::vector<std::pair<std::string, std::string>> device_info;
    WSADATA wsaData;

    // Initialize Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        throw std::runtime_error("Error initializing Winsock");
    }

    // Create a UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET) {
        throw std::runtime_error("Error creating socket");
    }

    // Enable broadcast for the socket
    BOOL broadcast_enable = TRUE;
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, (char*)&broadcast_enable, sizeof(broadcast_enable)) == SOCKET_ERROR) {
        throw std::runtime_error("Error enabling broadcast");
    }

    // Prepare the broadcast address
    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(12345);  // Choose an appropriate port number
    broadcast_addr.sin_addr.S_un.S_addr = INADDR_BROADCAST;

    // Send a broadcast packet
    const char* broadcast_message = "Device ID request";
    if (sendto(sockfd, broadcast_message, strlen(broadcast_message), 0, (sockaddr*)&broadcast_addr, sizeof(broadcast_addr)) == SOCKET_ERROR) {
        throw std::runtime_error("Error sending broadcast packet");
    }

    // Listen for responses for a short period of time
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Receive packets and process device IDs and IP addresses
    char buffer[1024];
    sockaddr_in device_addr;
    int device_addr_len = sizeof(device_addr);
    int recv_len;

    set_socket_timeout(sockfd, 5);
    while ((recv_len = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0, (sockaddr*)&device_addr, &device_addr_len)) != SOCKET_ERROR) {
        std::cout << "hello" << std::endl;
        buffer[recv_len] = '\0';

        // Extract the device ID from the received packet
        std::string device_id(buffer);

        // Extract the IP address from the received packet
        char ip_buffer[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &device_addr.sin_addr, ip_buffer, INET_ADDRSTRLEN);
        std::string ip_address(ip_buffer);

        // Add the device ID and IP address to the device_info vector
        device_info.emplace_back(device_id, ip_address);
        std::cout << "Device ID: " << device_id << ", IP address: " << ip_address << std::endl;
    }
    std::cout << "end" << std::endl;
    // Close the socket
    closesocket(sockfd);

    // Cleanup Winsock
    WSACleanup();

    return device_info;
}

void NetworkUtilLibrary::set_socket_timeout(int sockfd, int seconds) {
    struct timeval timeout;
    timeout.tv_sec = seconds;
    timeout.tv_usec = 0;

    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout)) < 0) {
        char error_msg[256];
        strerror_s(error_msg, sizeof(error_msg), errno);
        std::cerr << "Error setting socket timeout: " << error_msg << std::endl;
    }
}

bool NetworkUtilLibrary::isValidIPAddressAndPort(const std::string& input, int port)
{
    // Check if the port number is within the valid range (1-65535)
    if (port < 1 || port > 65535) {
        return false;
    }

    std::stringstream ss(input);
    std::string segment;
    int numSegments = 0;

    while (std::getline(ss, segment, '.')) {
        // Each segment should have at most 3 characters
        if (segment.size() > 3) {
            return false;
        }

        // If there is a leading zero and the segment is not "0", it's invalid
        if (segment.size() > 1 && segment[0] == '0') {
            return false;
        }

        // Each character in the segment should be a digit
        for (const char& c : segment) {
            if (!std::isdigit(c)) {
                return false;
            }
        }

        // Each segment should be within the range of 0-255
        int num = std::stoi(segment);
        if (num < 0 || num > 255) {
            return false;
        }

        numSegments++;
    }

    // An IPv4 address should have exactly 4 segments
    return numSegments == 4;
}
