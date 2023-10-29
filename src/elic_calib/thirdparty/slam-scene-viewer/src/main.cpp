//
// Created by csl on 10/16/22.
//
#include "slam-scene-viewer/scene_viewer.h"
#include "pcl/io/pcd_io.h"

int main(int argc, char **argv) {
    try {
        using namespace ns_viewer;
        SceneViewer viewer("../scene-shot");

        CubePlane plane(0.0f, 0.0f, -30.0f, 1.0f, 2.0f, 1.0f);
        viewer.AddCubePlane(plane, true);
        viewer.AddFeatures(plane.GenerateFeatures(10, CubePlane::ALL, 1.0f), 6.0f);
        viewer.AddPose(plane.LtoW);

        Eigen::AngleAxisf r1(-M_PI_4, Eigen::Vector3f(0, 0, 1));
        Eigen::AngleAxisf r2(+M_PI * 0.9, Eigen::Vector3f(1, 0, 0));
        auto rot = (r2 * r1).toRotationMatrix();
        auto CtoW = Posef(rot.inverse(), Eigen::Vector3f(2, -2, 10));
        auto LtoW = Posef(rot.inverse(), Eigen::Vector3f(2, 2, 1));
        auto ItoW = Posef(rot.inverse(), Eigen::Vector3f(2, 2, 2));

        viewer.AddCamera(CtoW, Colour::Green());
        viewer.AddLiDAR(LtoW, ns_viewer::Colour::Blue());
        viewer.AddIMU(ItoW, ns_viewer::Colour::Red());

        viewer.AddLine(Eigen::Vector3f{1, 2, 1}, Eigen::Vector3f{0, 0, 0});
        viewer.AddArrow(
                Eigen::Vector3f{CtoW.translation(0), CtoW.translation(1), CtoW.translation(2)},
                Eigen::Vector3f{ItoW.translation(0), ItoW.translation(1), ItoW.translation(2)},
                ns_viewer::Colour(0.3f, 0.3f, 0.3f, 0.5f)
        );
        viewer.AddArrow(
                Eigen::Vector3f{LtoW.translation(0), LtoW.translation(1), LtoW.translation(2)},
                Eigen::Vector3f{ItoW.translation(0), ItoW.translation(1), ItoW.translation(2)},
                ns_viewer::Colour(0.3f, 0.3f, 0.3f, 0.5f)
        );

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile("/home/csl/CppWorks/artwork/slam-scene-viewer/data/scan.pcd", *cloud);
        viewer.AddScan<pcl::PointXYZI>(
                cloud, 2.0f, true, ns_viewer::SceneViewer::GetUniqueColour().WithAlpha(0.1f)
        );

        viewer.AddBox(Eigen::Vector3f(-1.0f, -1.0f, -1.0f), Eigen::Vector3f(1.0f, 1.0f, 1.0f),
                      SceneViewer::GetUniqueColour().WithAlpha(0.3f));

        viewer.AddSphere({0, 0, 0}, 1.0f, ns_viewer::Colour::Red().WithAlpha(0.2f), false);

        viewer.SetViewPort(CtoW);
        viewer.RunMultiThread();
        std::cout << "hello, world!" << std::endl;

    } catch (const std::exception &e) {
        std::cout << e.what() << std::endl;
    }
    return 0;
}