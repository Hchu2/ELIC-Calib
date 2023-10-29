//
// Created by csl on 10/22/22.
//

#include "slam-scene-viewer/scene_viewer.h"

namespace ns_viewer {
    ColourWheel SceneViewer::COLOUR_WHEEL = ColourWheel(1.0f);

    std::size_t SceneViewer::CUBE_PLANE_COUNT = 0;
    std::size_t SceneViewer::FEATURE_COUNT = 0;
    std::size_t SceneViewer::POSE_COUNT = 0;
    std::size_t SceneViewer::LINE_COUNT = 0;
    std::size_t SceneViewer::ARROW_COUNT = 0;
    std::size_t SceneViewer::SCAN_COUNT = 0;

    std::size_t SceneViewer::CAMERA_COUNT = 0;
    std::size_t SceneViewer::LiDAR_COUNT = 0;
    std::size_t SceneViewer::IMU_COUNT = 0;

    const std::string SceneViewer::SHAPE_PREFIX = "S";
    const std::string SceneViewer::POINT_CLOUD_PREFIX = "P";
    const std::string SceneViewer::COORD_PREFIX = "C";

    SceneViewer::~SceneViewer() {
        if (_thread != nullptr) {
            _thread->join();
        }
    }

    pcl::visualization::PCLVisualizer::Ptr SceneViewer::GetViewer() {
        return _viewer;
    }

    void SceneViewer::SetColourWheel(const ColourWheel &colourWheel) {
        SceneViewer::COLOUR_WHEEL = colourWheel;
    }

    ColourWheel &SceneViewer::GetColourWheel() {
        return COLOUR_WHEEL;
    }

    Colour SceneViewer::GetUniqueColour() {
        return COLOUR_WHEEL.GetUniqueColour();
    }

    void SceneViewer::SetWindowName(const std::string &name) {
        _viewer->setWindowName(name);
    }

    SceneViewer &SceneViewer::operator()(const std::string &name) {
        this->SetWindowName(name);
        return *this;
    }

    std::mutex &SceneViewer::GetMutex() {
        return _mt;
    }

    void SceneViewer::Lock() {
        _mt.lock();
    }

    void SceneViewer::UnLock() {
        _mt.unlock();
    }

    void SceneViewer::RunSingleThread(int time) {
        if (std::filesystem::exists(_saveDir)) {
            std::cout
                    << "\033[92m\033[3m[SceneViewer] adjust the camera and press 'Alter' key to save the current scene.\033[0m"
                    << std::endl;
        }
        while (!_viewer->wasStopped()) {
            // ms
            _viewer->spinOnce(time);
        }
        RemoveEntities();
        InitSceneViewer();
    }

    void SceneViewer::RunMultiThread(int time) {
        if (std::filesystem::exists(_saveDir)) {
            std::cout
                    << "\033[92m\033[3m[SceneViewer] adjust the camera and press 'Alter' key to save the current scene.\033[0m"
                    << std::endl;
        }
        this->_thread = std::make_shared<std::thread>([this, time]() {
            while (!_viewer->wasStopped()) {
                Lock();
                _viewer->spinOnce(1);
                UnLock();

                // ms
                std::this_thread::sleep_for(std::chrono::milliseconds(time));
            }
        });
    }

    // ----------
    // add shapes
    // ----------

    std::vector<std::string> SceneViewer::AddCubePlane(const CubePlane &plane, bool lineMode) {
        std::vector<std::string> names;

        const auto name = GetShapeName("CUBE-PLANE-" + std::to_string(CUBE_PLANE_COUNT++));
        AppendNames(names, name);
        const auto &localToWorld = plane.LtoW;
        const auto &color = plane.color;

        Eigen::Vector3f localInWorld = localToWorld.translation;
        Eigen::Quaternionf Q_localToWorld(localToWorld.quaternion());

        _viewer->addCube(localInWorld, Q_localToWorld, plane.xSpan, plane.ySpan, plane.zSpan, name);

        if (lineMode) {
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_REPRESENTATION,
                    pcl::visualization::RenderingRepresentationProperties::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name
            );
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 2.0, name
            );
        }

        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, color.a, name
        );

        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR,
                color.r, color.g, color.b, name
        );
        return names;
    }

    std::vector<std::string>
    SceneViewer::AddBox(const Vector3f &boxMin, const Vector3f &boxMax, const Colour &color, bool lineMode) {
        std::vector<std::string> names;

        const auto name = GetShapeName("BOX-" + std::to_string(CUBE_PLANE_COUNT++));
        AppendNames(names, name);

        _viewer->addCube(
                boxMin(0), boxMax(0), boxMin(1), boxMax(1), boxMin(2), boxMax(2),
                color.r, color.g, color.b, name
        );

        if (lineMode) {
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_REPRESENTATION,
                    pcl::visualization::RenderingRepresentationProperties::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                    name
            );
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 2.0, name
            );
        }
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, color.a, name
        );

        return names;
    }

    std::vector<std::string>
    SceneViewer::AddSphere(const Vector3f &center, float radius, const Colour &color, bool lineMode) {
        std::vector<std::string> names;

        const auto name = GetShapeName("SPHERE-" + std::to_string(CUBE_PLANE_COUNT++));
        AppendNames(names, name);

        _viewer->addSphere<pcl::PointXYZ>(
                {center(0), center(1), center(2)}, radius, color.r, color.g, color.b, name
        );

        if (lineMode) {
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_REPRESENTATION,
                    pcl::visualization::RenderingRepresentationProperties::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                    name
            );
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 2.0, name
            );
        }
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, color.a, name
        );

        return names;
    }

    std::vector<std::string>
    SceneViewer::AddFeatures(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &features, float size) {
        std::vector<std::string> names;

        const auto name = GetPointCloudName("FEATURE-" + std::to_string(FEATURE_COUNT++));
        AppendNames(names, name);

        _viewer->addPointCloud(features, name);
        _viewer->setPointCloudRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, size, name
        );
        return names;
    }

    std::vector<std::string> SceneViewer::AddPose(const Posef &LtoW, float size) {
        std::vector<std::string> names;

        const auto name = GetCoordName("POSE-" + std::to_string(POSE_COUNT++));
        AppendNames(names, name);


        Eigen::Isometry3f localToWorld(LtoW.quaternion());
        localToWorld.pretranslate(LtoW.translation);

        _viewer->addCoordinateSystem(
                size, Eigen::Affine3f(localToWorld.affine()), name
        );

        return names;
    }

    std::vector<std::string> SceneViewer::AddCamera(const Posef &CtoW, const Colour &color, float size) {
        std::vector<std::string> names;

        const auto name = "CAMERA-" + std::to_string(CAMERA_COUNT++);
        AppendNames(names, AddPose(CtoW, size * 0.75f));

        float left = -0.8f * size, right = 0.8f * size, top = -0.6f * size, bottom = 0.6f * size, front = 0.8f * size;

        pcl::PointXYZ leftBottom = {left, bottom, front};
        pcl::PointXYZ leftTop = {left, top, front};
        pcl::PointXYZ rightTop = {right, top, front};
        pcl::PointXYZ rightBottom = {right, bottom, front};
        pcl::PointXYZ center = {0.0,
                                0.0,
                                0.0};

        std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> data{
                {leftBottom,  rightBottom},
                {leftTop,     rightTop},
                {leftTop,     leftBottom},
                {rightTop,    rightBottom},
                {leftTop,     center},
                {leftBottom,  center},
                {rightTop,    center},
                {rightBottom, center},
        };
        for (auto &[p1, p2]: data) {

            auto r1 = CtoW.trans(Eigen::Vector3f(p1.x, p1.y, p1.z));
            p1.x = r1(0), p1.y = r1(1), p1.z = r1(2);

            auto r2 = CtoW.trans(Eigen::Vector3f(p2.x, p2.y, p2.z));
            p2.x = r2(0), p2.y = r2(1), p2.z = r2(2);

            AppendNames(names, AddLine(p1, p2, color, size * 4.0f));
        }

        return names;
    }

    std::vector<std::string>
    SceneViewer::AddLine(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const Colour &color, float size) {
        std::vector<std::string> names;

        const auto name = GetShapeName("LINE-" + std::to_string(LINE_COUNT++));
        AppendNames(names, name);

        _viewer->addLine(p1, p2, color.r, color.g, color.b, name);
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, size, name
        );
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, color.a, name
        );
        return names;
    }

    std::vector<std::string>
    SceneViewer::AddLine(const Vector3f &p1, const Vector3f &p2, const Colour &color, float size) {
        return AddLine(EigenVecToPointXYZ(p1), EigenVecToPointXYZ(p2), color, size);
    }

    std::vector<std::string>
    SceneViewer::AddArrow(const pcl::PointXYZ &from, const pcl::PointXYZ &to, const Colour &color, float size) {
        std::vector<std::string> names;

        AppendNames(names, AddLine(from, to, color, size * 2.0f));

        const auto name = GetShapeName("ARROW-" + std::to_string(ARROW_COUNT++));

        pcl::ModelCoefficients coneCoeff;
        coneCoeff.values.resize(7);
        coneCoeff.values[0] = to.x;
        coneCoeff.values[1] = to.y;
        coneCoeff.values[2] = to.z;
        float dist = std::sqrt(
                std::pow(from.x - to.x, 2.0f) + std::pow(from.y - to.y, 2.0f) + std::pow((from.z - to.z), 2.0f));
        coneCoeff.values[3] = size * 0.05f * (from.x - to.x) / dist;
        coneCoeff.values[4] = size * 0.05f * (from.y - to.y) / dist;
        coneCoeff.values[5] = size * 0.05f * (from.z - to.z) / dist;
        coneCoeff.values[6] = 25.0f;
        _viewer->addCone(coneCoeff, name);

        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR,
                color.r, color.g, color.b, name
        );
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, color.a, name
        );

        return names;
    }

    std::vector<std::string>
    SceneViewer::AddArrow(const Vector3f &from, const Vector3f &to, const Colour &color, float size) {
        return AddLine(EigenVecToPointXYZ(from), EigenVecToPointXYZ(to), color, size);
    }

    std::vector<std::string> SceneViewer::AddLiDAR(const Posef &LtoW, const Colour &color, float size) {
        std::vector<std::string> names;

        AppendNames(names, AddPose(LtoW, size * 0.75f));

        pcl::ModelCoefficients cylinderCoeff;
        cylinderCoeff.values.resize(7);
        cylinderCoeff.values[0] = LtoW.translation(0) - size * 0.75f * LtoW.rotation(0, 2);
        cylinderCoeff.values[1] = LtoW.translation(1) - size * 0.75f * LtoW.rotation(1, 2);
        cylinderCoeff.values[2] = LtoW.translation(2) - size * 0.75f * LtoW.rotation(2, 2);

        cylinderCoeff.values[3] = size * 1.5f * LtoW.rotation(0, 2);
        cylinderCoeff.values[4] = size * 1.5f * LtoW.rotation(1, 2);
        cylinderCoeff.values[5] = size * 1.5f * LtoW.rotation(2, 2);

        cylinderCoeff.values[6] = size;

        auto name = GetShapeName("LiDAR-" + std::to_string(LiDAR_COUNT++));

        _viewer->addCylinder(cylinderCoeff, name);

        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::RenderingRepresentationProperties::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name
        );
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 2.0, name
        );
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR,
                color.r, color.g, color.b, name
        );
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, color.a, name
        );

        AppendNames(names, name);

        return names;
    }

    std::vector<std::string> SceneViewer::AddIMU(const Posef &LtoW, const Colour &color, float size) {
        std::vector<std::string> names;

        AppendNames(names, AddPose(LtoW, size * 0.75f));

        Eigen::Vector3f localInWorld = LtoW.translation;
        Eigen::Quaternionf Q_localToWorld(LtoW.quaternion());

        auto name = GetShapeName("IMU-" + std::to_string(IMU_COUNT++));
        AppendNames(names, name);

        _viewer->addCube(localInWorld, Q_localToWorld, size, size, size, name);

        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::RenderingRepresentationProperties::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name
        );
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 2.0, name
        );
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, color.a, name
        );
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR,
                color.r, color.g, color.b, name
        );
        return names;
    }

    void SceneViewer::RemoveEntities(const std::string &name) {
        if (name.empty()) {
            return;
        } else {
            if (name.front() == SHAPE_PREFIX.front()) {
                _viewer->removeShape(name);
            } else if (name.front() == COORD_PREFIX.front()) {
                _viewer->removeCoordinateSystem(name);
            } else if (name.front() == POINT_CLOUD_PREFIX.front()) {
                _viewer->removePointCloud(name);
            }
        }
    }

    void SceneViewer::RemoveEntities(const std::vector<std::string> &names) {
        for (const auto &item: names) {
            RemoveEntities(item);
        }
    }

    void SceneViewer::RemoveEntities(const std::vector<std::vector<std::string>> &names) {
        for (const auto &items: names) {
            RemoveEntities(items);
        }
    }

    void SceneViewer::RemoveEntities() {
        _viewer->removeAllShapes();
        _viewer->removeAllPointClouds();
        _viewer->removeAllCoordinateSystems();
    }

    // -------
    // helpers
    // -------

    void SceneViewer::KeyBoardCallBack(const pcl::visualization::KeyboardEvent &ev) {
        if (ev.isAltPressed()) {
            std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
            const std::string filename = _saveDir + "/" + std::to_string(curTimeStamp) + ".png";
            _viewer->saveScreenshot(filename);
            std::cout << "\033[92m\033[3m[SceneViewer] the scene shot is saved to path: '" << filename << "\033[0m"
                      << std::endl;
        }
    }

    void SceneViewer::InitSceneViewer() {
        _viewer->setBackgroundColor(_bgc.r, _bgc.g, _bgc.b);
        // coordinates
        if (_addOriginCoord) {
            _viewer->addCoordinateSystem(1.0, "Origin");
        }
        if (std::filesystem::exists(_saveDir)) {
            // shot
            using std::placeholders::_1;
            _viewer->registerKeyboardCallback(
                    [this](auto &&PH1) { KeyBoardCallBack(std::forward<decltype(PH1)>(PH1)); }
            );
        }
    }

    void SceneViewer::AppendNames(std::vector<std::string> &names, const std::string &name) {
        names.push_back(name);
    }

    void SceneViewer::AppendNames(std::vector<std::string> &names, const std::vector<std::string> &newNames) {
        names.insert(names.end(), newNames.cbegin(), newNames.cend());
    }

    std::string SceneViewer::GetShapeName(const std::string &desc) {
        return SHAPE_PREFIX + '-' + desc;
    }

    std::string SceneViewer::GetPointCloudName(const std::string &desc) {
        return POINT_CLOUD_PREFIX + '-' + desc;
    }

    std::string SceneViewer::GetCoordName(const std::string &desc) {
        return COORD_PREFIX + '-' + desc;
    }

    Eigen::Vector3f SceneViewer::PointXYZtoEigenVec(const pcl::PointXYZ &p) {
        return {p.x, p.y, p.z};
    }

    pcl::PointXYZ SceneViewer::EigenVecToPointXYZ(const Vector3f &p) {
        return {p(0), p(1), p(2)};
    }

    void SceneViewer::SetViewPort(const Posef &pose, int width, int height) {
        const Eigen::Matrix3f rot = pose.rotation;
        const Eigen::Vector3f xAxis = rot.col(0);
        const Eigen::Vector3f yAxis = rot.col(1);
        const Eigen::Vector3f zAxis = rot.col(2);
        const Eigen::Vector3f t = pose.translation;

        _viewer->setCameraPosition(
                t(0), t(1), t(2),
                t(0) + zAxis(0), t(1) + zAxis(1), t(2) + zAxis(2),
                -yAxis(0), -yAxis(1), -yAxis(2)
        );
        _viewer->setSize(width, height);
    }
}

