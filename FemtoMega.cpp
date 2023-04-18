#include "FemtoMega.h"
#include "NetworkUtilLibrary.h"


bool FemtoMega::init(const std::vector<std::pair<std::string, int>>& input_ip_address_and_ports)
{
    // Setup devices
    for (std::pair<std::string, int> ip_address_and_port : input_ip_address_and_ports)
    {
        // Check if ip address and port is valid
        std::string ip = ip_address_and_port.first;
        int port = ip_address_and_port.second;
        if (!NetworkUtilLibrary::getInstance().isValidIPAddressAndPort(ip, port)) continue;

        // Create device
        std::shared_ptr<ob::Device> device = ctx.createNetDevice(ip.c_str(), port);
        if (!device) continue;
        devices.push_back(device);

        // Show device info from context
        std::shared_ptr<ob::DeviceInfo> deviceInfo = device->getDeviceInfo();
        if (!deviceInfo) {
            std::cout << "Failed to get device info" << std::endl;
        }
        // Device name, pid, vid, uid, serial number, connection type
        std::cout << "------------" << std::endl;
        std::cout << "Device name: " << deviceInfo->name() << std::endl;
        std::cout << "Device pid: " << deviceInfo->pid() << std::endl;
        std::cout << "Device vid: " << deviceInfo->vid() << std::endl;
        std::cout << "Device uid: " << deviceInfo->uid() << std::endl;
        std::cout << "Device serial number: " << deviceInfo->serialNumber() << std::endl;
        std::cout << "Device connection type: " << deviceInfo->connectionType() << std::endl;
        std::cout << "------------" << std::endl;
    }

    // Memo
    // for Sync
    // 
    // if you don't use configuration file, need these process
    // 1. Load the configuration file and configure the device;
    // 2. Reboot devices;
    // 
    // 3. Wait for all devices to reboot complete, start all devices's color and depth sensor to handle it frames;
    
    // 1. Load the configuration file and configure all devices
    // 2. Reboot

    // I don't know if this needed, becouse ctx returns no device if it's network one.
    ctx.enableMultiDeviceSync(60000);
    // 3. start all devices's color and depth sensor to handle it frames
    for (int i = 0; i < devices.size(); i++)
    {
        // Set sync config
        OBDeviceSyncConfig syncConfig;
        if (i == 0)
        {
            syncConfig.syncMode = OBSyncMode::OB_SYNC_MODE_STANDALONE;
		}
		else
		{
            syncConfig.syncMode = OBSyncMode::OB_SYNC_MODE_STANDALONE;
        }
        //devices[i]->setSyncConfig(syncConfig);

        // Setup pipeline
        std::shared_ptr<ob::Pipeline> pipeline = std::make_shared<ob::Pipeline>(devices[i]);
        if (!pipeline) {
            std::cout << "Failed to create pipeline" << std::endl;
            return false;
        }
        pipelines.push_back(pipeline);

        // Setup config
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
        if (!config) {
            std::cout << "Failed to create config" << std::endl;
            return false;
        }

        // Depth
        // Setup depth profileList and depthProfile, if enable, set config enableStream
        std::shared_ptr<ob::StreamProfileList> depthProfileList = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
        if (!depthProfileList) {
            std::cout << "Failed to get depth profile list" << std::endl;
            return false;
        }
        std::shared_ptr<ob::StreamProfile> depthProfile = depthProfileList->getProfile(0);
        if (!depthProfile) {
            std::cout << "Failed to get depth profile" << std::endl;
            return false;
        }
        config->enableStream(depthProfile);


        // Color
        // Setup color profileList and colorProfile, if enable, set config enableStream
        std::shared_ptr<ob::StreamProfileList> colorProfileList = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
        if (!colorProfileList) {
            std::cout << "Failed to get color profile list" << std::endl;
            return false;
        }
        std::shared_ptr<ob::StreamProfile> colorProfile = colorProfileList->getProfile(0);
        if (!colorProfile) {
            std::cout << "Failed to get color profile" << std::endl;
            return false;
        }
        config->enableStream(colorProfile);

    }


	return true;
}

void FemtoMega::start()
{
    // start stream
    std::cout << "start stream" << std::endl;
    startStream(devices, OB_SENSOR_COLOR, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    startStream(devices, OB_SENSOR_DEPTH, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void FemtoMega::stop()
{
    // stop stream
    std::cout << "stop stream" << std::endl;
    stopStream(devices, OB_SENSOR_COLOR, 0);
    stopStream(devices, OB_SENSOR_DEPTH, 0);
}

void FemtoMega::startStream(std::vector<std::shared_ptr<ob::Device>> devices, OBSensorType sensorType, int deviceIndexBase) {
    for (auto&& dev : devices) {
        // Get sensor list from device
        auto sensorList = dev->getSensorList();
        for (uint32_t i = 0; i < sensorList->count(); i++) {
            auto sensor = sensorList->getSensor(i);
            if (sensorType == sensor->type()) {
                auto profiles = sensor->getStreamProfileList();
                auto profile = profiles->getProfile(0);
                switch (sensorType) {
                case OB_SENSOR_DEPTH:
                    if (profile) {
                        sensor->start(profile, [this, deviceIndexBase](std::shared_ptr<ob::Frame> frame) { this->handleDepthStream(deviceIndexBase, frame); });
                    }
                    break;
                case OB_SENSOR_COLOR:
                    if (profile) {
                        sensor->start(profile, [this, deviceIndexBase](std::shared_ptr<ob::Frame> frame) { this->handleColorStream(deviceIndexBase, frame); });
                    }
                    break;
                default:
                    break;
                }
            }
        }
        deviceIndexBase++;
    }
}

void FemtoMega::stopStream(std::vector<std::shared_ptr<ob::Device>> devices, OBSensorType sensorType, int deviceIndexBase)
{
    for (auto&& dev : devices) {
        // Get sensor list from device
        auto sensorList = dev->getSensorList();
        for (uint32_t i = 0; i < sensorList->count(); i++) {
            if (sensorList->type(i) == sensorType) {
                sensorList->getSensor(i)->stop();
                break;
            }
        }
    }
}

// Callbacks
void FemtoMega::handleColorStream(int devIndex, std::shared_ptr<ob::Frame> frame) {
    std::lock_guard<std::mutex> lock(frameMutex);
    std::cout << "Device#" << devIndex << ", color frame index=" << frame->index() << ", timestamp=" << frame->timeStamp()
        << ", system timestamp=" << frame->systemTimeStamp() << std::endl;

    //if (colorFrames[devIndex])
    //{
    //    colorFrames[devIndex] = frame;
    //}
    //else {
    //    colorFrames.push_back(frame);
    //}
    
}

void FemtoMega::handleDepthStream(int devIndex, std::shared_ptr<ob::Frame> frame) {
    std::lock_guard<std::mutex> lock(frameMutex);
    std::cout << "Device#" << devIndex << ", depth frame index=" << frame->index() << ", timestamp=" << frame->timeStamp()
        << ", system timestamp=" << frame->systemTimeStamp() << std::endl;

    //if (depthFrames[devIndex])
    //{
    //    depthFrames[devIndex] = frame;
    //}
    //else {
    //    depthFrames.push_back(frame);
    //}
}