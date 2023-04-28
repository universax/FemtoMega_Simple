#include "FemtoMega.h"
#include "Util.h"

#pragma region Public
bool FemtoMega::init(const std::vector<std::pair<std::string, int>>& input_ip_address_and_ports)
{
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_ERROR);

    initSensors(input_ip_address_and_ports);
    initPointCloud();

	return true;
}

void FemtoMega::update()
{
    updateFrameSet();
    updateColorDepth();
    updatePointCloud();
}

void FemtoMega::draw()
{
    //drawColorMat();
    drawDepthMat();
}

void FemtoMega::stop()
{
    // stop stream
    std::cout << "stop stream" << std::endl;
    for (auto&& pipeline : pipelines) {
        pipeline->stop();
    }
}
#pragma endregion

#pragma region Private
#pragma region Init
// Private --------------------------------------
// init
bool FemtoMega::initSensors(const std::vector<std::pair<std::string, int>>& input_ip_address_and_ports)
{
    // Setup devices
    for (std::pair<std::string, int> ip_address_and_port : input_ip_address_and_ports)
    {
        // Check if ip address and port is valid
        std::string ip = ip_address_and_port.first;
        int port = ip_address_and_port.second;
        //if (!NetworkUtilLibrary::getInstance().isValidIPAddressAndPort(ip, port)) continue;
        // Create device
        std::shared_ptr<ob::Device> device = ctx.createNetDevice(ip.c_str(), port);
        if (!device) continue;

        // Show device info from context
        showDeviceInfo(device);

        //// Set sync config
        //OBDeviceSyncConfig syncConfig;
        //if (i == 0)
        //{
        //    syncConfig.syncMode = OBSyncMode::OB_SYNC_MODE_STANDALONE;
        //}
        //else
        //{
        //    syncConfig.syncMode = OBSyncMode::OB_SYNC_MODE_STANDALONE;
        //}
        //devices[i]->setSyncConfig(syncConfig);

        // Setup pipeline
        std::shared_ptr<ob::Pipeline> pipeline = std::make_shared<ob::Pipeline>(device);
        pipelines.push_back(pipeline);

        // Depth
        // Setup depth profileList and depthProfile, if enable, set config enableStream
        std::shared_ptr<ob::StreamProfileList> depthProfileList = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
        try
        {
            depthStreamProfile = depthProfileList->getVideoStreamProfile(640/2, 576/2, OBFormat::OB_FORMAT_Y16, 30);
        }
        catch (const std::exception&)
        {
            depthStreamProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfileList->getProfile(0))->as<ob::VideoStreamProfile>();
        }
        depthMats.push_back(cv::Mat());

        // Color
        // Setup color profileList and colorProfile, if enable, set config enableStream
        std::shared_ptr<ob::StreamProfileList> colorProfileList = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
        try
        {
            colorStreamProfile = colorProfileList->getVideoStreamProfile(1920, 1080, OBFormat::OB_FORMAT_H264, 30);
        }
        catch (const std::exception&)
        {
            colorStreamProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfileList->getProfile(0))->as<ob::VideoStreamProfile>();
        }
        colorMats.push_back(cv::Mat());

        // Set Stream Profile
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
        config->enableStream(depthStreamProfile);
        config->enableStream(colorStreamProfile);

        // Set Align Mode
        config->setAlignMode(OBAlignMode::ALIGN_D2C_HW_MODE);
        if (format == OBFormat::OB_FORMAT_POINT)
        {
            config->setAlignMode(OBAlignMode::ALIGN_DISABLE);
        }

        // Start pipeline
        pipeline->start(config);
    }
    return true;
}

void FemtoMega::initPointCloud()
{
    // Create Point Cloud filter
    pointcloudFilters.clear();
    for (size_t i = 0; i < pipelines.size(); i++)
    {
        const OBCameraParam cameraParameter = pipelines[i]->getCameraParam();
        std::shared_ptr<ob::PointCloudFilter> pointcloudFilter = std::make_shared<ob::PointCloudFilter>();
        pointcloudFilter->setCameraParam(cameraParameter);
        pointcloudFilter->setCreatePointFormat(format);
        pointcloudFilter->setColorDataNormalization(true);
        pointcloudFilters.push_back(pointcloudFilter);

        // Create Point Cloud
        pcl::PointCloud<PointType>::Ptr pointcloud(new pcl::PointCloud<PointType>());
        pointclouds.push_back(pointcloud);
    }
}
#pragma endregion

#pragma region Update
// update
void FemtoMega::updateFrameSet()
{
    framesets.clear();
    for (size_t i = 0; i < pipelines.size(); i++)
    {
        constexpr int32_t timeout = std::chrono::milliseconds(100).count();
        framesets.push_back(pipelines[i]->waitForFrames(timeout));
    }
}

void FemtoMega::updateColorDepth()
{
    colorFrames.clear();
    depthFrames.clear();
    for (size_t i = 0; i < framesets.size(); i++)
    {
        colorFrames.push_back(framesets[i]->colorFrame());
        depthFrames.push_back(framesets[i]->depthFrame());
    }
}

void FemtoMega::updatePointCloud()
{
    //visualizer.ClearGeometries();
    for (size_t i = 0; i < pointcloudFilters.size(); i++)
    {
        // null check
        if (framesets[i] == nullptr) continue;
        if (framesets[i]->colorFrame() == nullptr || framesets[i]->depthFrame() == nullptr) continue;

        // Create Pointcloud from frameset
        std::shared_ptr<ob::Frame> pointcloudFrame = pointcloudFilters[i]->process(framesets[i]);

        // Create Pointcloud for PCL
        pcl::PointCloud<PointType>::Ptr pointcloud(new pcl::PointCloud<PointType>());
        if (format == OBFormat::OB_FORMAT_RGB_POINT)
        {
            const int32_t numPoints = pointcloudFrame->dataSize() / sizeof(OBColorPoint);
            OBColorPoint* data = reinterpret_cast<OBColorPoint*>(pointcloudFrame->data());

            for (int32_t j = 0; j < numPoints; j++)
            {
                // Set point to pointcloud
                PointType p(data[j].x * 0.001, data[j].y * 0.001, data[j].z * 0.001, data[j].r, data[j].g, data[j].b);
                pointcloud->points.push_back(p);
            }
        }
        else
        {
            const int32_t numPoints = pointcloudFrame->dataSize() / sizeof(OBPoint);
            OBPoint* data = reinterpret_cast<OBPoint*>(pointcloudFrame->data());

            srand((i+1)*200);
            int32_t r = rand() % 256;
            int32_t g = rand() % 256;
            int32_t b = rand() % 256;

            for (int32_t j = 0; j < numPoints; j++)
            {
                if (data[j].x == 0 && data[j].y == 0 && data[j].z == 0) continue;

                // Set point to pointcloud
                PointType p(data[j].x * 0.001, data[j].z * 0.001, data[j].y * 0.001, r, g, b);
                pointcloud->points.push_back(p);
            }
        }
        // Set pointcloud to vector
        *pointclouds[i] = *pointcloud;
    }
}
#pragma endregion

#pragma region Draw
// draw
void FemtoMega::drawColorMat()
{
    for (size_t i = 0; i < pipelines.size(); i++)
    {
        // Color
        if (colorFrames[i] == nullptr) continue;
        colorMats[i] = ob::get_mat(colorFrames[i]);
        if (!colorMats[i].empty())
        {
            cv::imshow("color_" + std::to_string(i), colorMats[i]);
        }
    }
}
void FemtoMega::drawDepthMat()
{
    for (size_t i = 0; i < pipelines.size(); i++)
    {
        // Depth
        if (depthFrames[i] == nullptr)  continue;
        depthMats[i] = ob::get_mat(depthFrames[i]);

        if (!depthMats[i].empty())
        {
            cv::imshow("depth_" + std::to_string(i), depthMats[i]);
        }
    }
}
#pragma endregion

#pragma region Util
// util
void FemtoMega::showDeviceInfo(const std::shared_ptr<ob::Device> device)
{
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

#pragma endregion