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

void FemtoMega::process()
{
    processPointCloud();
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

void FemtoMega::registration()
{
    registrationPointcloud();
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
    // Create visualizer
    const int32_t width = 1280;
    const int32_t height = 720;


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
        // Color
        colorFrames.push_back(framesets[i]->colorFrame());
        // Depth
        depthFrames.push_back(framesets[i]->depthFrame());
    }
}

void FemtoMega::updatePointCloud()
{
    ////visualizer.ClearGeometries();
    //for (size_t i = 0; i < pointcloudFilters.size(); i++)
    //{
    //    // null check
    //    if (framesets[i] == nullptr)
    //    {
    //        continue;
    //    }
    //    if (framesets[i]->colorFrame() == nullptr || framesets[i]->depthFrame() == nullptr)
    //    {
    //        continue;
    //    }

    //    // Create Pointcloud from frameset
    //    std::shared_ptr<ob::Frame> pointcloudFrame = pointcloudFilters[i]->process(framesets[i]);

    //    // Create Pointcloud for Open3D
    //    if (format == OBFormat::OB_FORMAT_RGB_POINT)
    //    {
    //        const int32_t numPoints = pointcloudFrame->dataSize() / sizeof(OBColorPoint);
    //        OBColorPoint* data = reinterpret_cast<OBColorPoint*>(pointcloudFrame->data());

    //        std::vector<Eigen::Vector3d> points = std::vector<Eigen::Vector3d>(numPoints);
    //        std::vector<Eigen::Vector3d> colors = std::vector<Eigen::Vector3d>(numPoints);

    //        for (int32_t i = 0; i < numPoints; i++)
    //        {
    //            points[i] = Eigen::Vector3d(data[i].x, data[i].y, data[i].z);
    //            colors[i] = Eigen::Vector3d(data[i].r / 255.0, data[i].g / 255.0, data[i].b / 255.0);
    //        }

    //    }
    //    else
    //    {
    //        const int32_t numPoints = pointcloudFrame->dataSize() / sizeof(OBPoint);
    //        OBPoint* data = reinterpret_cast<OBPoint*>(pointcloudFrame->data());

    //        std::vector<Eigen::Vector3d> points = std::vector<Eigen::Vector3d>(numPoints);

    //        #pragma omp parallel for
    //        for (int32_t i = 0; i < numPoints; i++)
    //        {
    //            points[i] = Eigen::Vector3d(data[i].x, data[i].y, data[i].z);
    //        }

    //    }
    //}
}
#pragma endregion

#pragma region Draw
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

}
#pragma endregion

#pragma region Process
// process
void FemtoMega::processPointCloud()
{
	//// registration
	//registrationPointcloud();
}

void FemtoMega::registrationPointcloud()
{
    // prepare
    // get downsampled pointcloud and FPFHFeature from that
    //double voxelSize = 0.2;
    //std::pair<std::shared_ptr<open3d::geometry::PointCloud>, std::shared_ptr<open3d::pipelines::registration::Feature>> srcPcdAndFeatuer, targetPcdAndFeature;
    //srcPcdAndFeatuer = calcFPFHFeature(pointclouds[0], voxelSize);
    //targetPcdAndFeature = calcFPFHFeature(pointclouds[1], voxelSize);
    //std::shared_ptr<open3d::geometry::PointCloud> srcPcdDownSampled = srcPcdAndFeatuer.first;
    //std::shared_ptr<open3d::geometry::PointCloud> targetPcdDownSampled = targetPcdAndFeature.first;
    //std::shared_ptr<open3d::pipelines::registration::Feature> srcFpfhFeature = srcPcdAndFeatuer.second;
    //std::shared_ptr<open3d::pipelines::registration::Feature> targetFpfhFeature = targetPcdAndFeature.second;

    //std::cout << "src downsampled pointcloud size: " << srcPcdDownSampled->points_.size() << std::endl;
    //std::cout << "target downsampled pointcloud size: " << targetPcdDownSampled->points_.size() << std::endl;

    //// global registration with RANSAC
    //open3d::pipelines::registration::RegistrationResult resultRANSAC = calcGlobalRegistration(
    //    srcPcdDownSampled,
    //    targetPcdDownSampled,
    //    srcFpfhFeature,
    //    targetFpfhFeature,
    //    voxelSize
    //);
    //std::cout << "::RANSAC result" << std::endl;
    //std::cout << resultRANSAC.transformation_ << std::endl;

 //   // refine registration with ICP
 //   open3d::pipelines::registration::RegistrationResult resultICP = calcRefineRegistration(
 //       pointclouds[0],
 //       pointclouds[1],
 //       resultRANSAC,
	//	voxelSize
	//);
 //   std::cout << resultICP.transformation_ << std::endl;
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
#pragma endregion
#pragma endregion