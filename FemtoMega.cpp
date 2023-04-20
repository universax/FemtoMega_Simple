#include "FemtoMega.h"
#include "Util.h"

bool FemtoMega::init(const std::vector<std::pair<std::string, int>>& input_ip_address_and_ports)
{
    initSensors(input_ip_address_and_ports);
    initPointCloud();

	return true;
}

void FemtoMega::update()
{
    updateColorDepth();
    updatePointCloud();
}

void FemtoMega::draw()
{
    drawColorDepth();
    drawPointcloud();
}

void FemtoMega::stop()
{
    // stop stream
    std::cout << "stop stream" << std::endl;
    for (auto&& pipeline : pipelines) {
        pipeline->stop();
    }
}

// Private -------------------
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
        devices.push_back(device);

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
            depthStreamProfile = depthProfileList->getVideoStreamProfile(320, 288, OBFormat::OB_FORMAT_Y16, 30);
        }
        catch (const std::exception&)
        {
            depthStreamProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfileList->getProfile(0))->as<ob::VideoStreamProfile>();
        }
        // Add frame to array
        depthFrames.push_back(nullptr);
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
        // Add frame to array
        colorFrames.push_back(nullptr);
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
    pointcloudFilter = std::make_shared<ob::PointCloudFilter>();
    const OBCameraParam cameraParameter = pipelines[0]->getCameraParam();
    pointcloudFilter->setCameraParam(cameraParameter);
    pointcloudFilter->setCreatePointFormat(format);
    pointcloudFilter->setColorDataNormalization(true);

    // Create PointCloud
    pointcloud = std::make_shared<open3d::geometry::PointCloud>();

    // Create visualizer
    const int32_t width = 1280;
    const int32_t height = 720;
    visualizer.CreateVisualizerWindow("Open3D", width, height);
    visualizer.RegisterKeyActionCallback(27,
        [&](open3d::visualization::Visualizer* visualizer, int a, int b) {
            return true;
        }
    );
}

// update
void FemtoMega::updateColorDepth()
{
    for (size_t i = 0; i < pipelines.size(); i++)
    {
        std::shared_ptr<ob::Pipeline> pipeline = pipelines[i];

        constexpr int32_t timeout = std::chrono::milliseconds(100).count();
        frameset = pipeline->waitForFrames(timeout);

        if (frameset == nullptr)
        {
            continue;
        }

        // Color
        colorFrames[i] = frameset->colorFrame();
        //
        // Depth
        depthFrames[i] = frameset->depthFrame();
    }
}

void FemtoMega::updatePointCloud()
{
    // null check
    if (frameset == nullptr)
    {
		return;
	}
    if (frameset->colorFrame() == nullptr || frameset->depthFrame() == nullptr)
    {
		return;
	}

    // Create Pointcloud from frameset
    pointcloudFrame = pointcloudFilter->process(frameset);
    if (pointcloudFrame == nullptr)
    {
		return;
	}

    // Create Pointcloud for Open3D
    if (format == OBFormat::OB_FORMAT_RGB_POINT)
    {
        const int32_t numPoints = pointcloudFrame->dataSize() / sizeof(OBColorPoint);
        OBColorPoint* data = reinterpret_cast<OBColorPoint*>(pointcloudFrame->data());

        std::vector<Eigen::Vector3d> points = std::vector<Eigen::Vector3d>(numPoints);
        std::vector<Eigen::Vector3d> colors = std::vector<Eigen::Vector3d>(numPoints);

        for (int32_t i = 0; i < numPoints; i++)
        {
            points[i] = Eigen::Vector3d(data[i].x, data[i].y, data[i].z);
            colors[i] = Eigen::Vector3d(data[i].r / 255.0, data[i].g / 255.0, data[i].b / 255.0);
        }

        pointcloud->points_ = points;
        pointcloud->colors_ = colors;
    }
    else
    {
        const int32_t numPoints = pointcloudFrame->dataSize() / sizeof(OBPoint);
        OBPoint* data = reinterpret_cast<OBPoint*>(pointcloudFrame->data());

        std::vector<Eigen::Vector3d> points = std::vector<Eigen::Vector3d>(numPoints);

#pragma omp parallel for
        for (int32_t i = 0; i < numPoints; i++)
        {
            points[i] = Eigen::Vector3d(data[i].x, data[i].y, data[i].z);
        }

        pointcloud->points_ = points;
    }
}

// draw
void FemtoMega::drawColorDepth()
{
    for (size_t i = 0; i < pipelines.size(); i++)
    {
        // Depth
        if (depthFrames[i] == nullptr)
        {
            continue;
        }
        depthMats[i] = ob::get_mat(depthFrames[i]);
        if (!depthMats[i].empty()) cv::imshow("depth_" + std::to_string(i), depthMats[i]);

        // Color
        //if (colorFrames[i] == nullptr)
        //{
        //    continue;
        //}
        //colorMats[i] = ob::get_mat(colorFrames[i]);
        //if (!colorMats[i].empty()) cv::imshow("color_" + std::to_string(i), colorMats[i]);
    }
}

void FemtoMega::drawPointcloud()
{
    if (pointcloudFrame == nullptr)
    {
        return;
    }

    if (!visualizer.HasGeometry())
    {
        visualizer.AddGeometry(pointcloud);
    }
    visualizer.UpdateGeometry(pointcloud);
    visualizer.PollEvents();
    visualizer.UpdateRender();
}

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

