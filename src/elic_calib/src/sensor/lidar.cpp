//
// Created by csl on 10/1/22.
//

#include "sensor/lidar.h"
#include "config/calib_config.h"
#include "thirdparty/logger/src/include/logger.h"

namespace ns_elic {

    LiDARFrame::LiDARFrame(double timestamp, PosTPointCloud::Ptr scan)
            : _scan(std::move(scan)),
              _timestamp(timestamp) {}

    LiDARFrame::Ptr
    LiDARFrame::Create(double timestamp, const PosTPointCloud::Ptr &scan) {
        return std::make_shared<LiDARFrame>(timestamp, scan);
    }

    double LiDARFrame::GetMaxTimestamp() const {
        return this->_scan->back().timestamp;
    }

    double LiDARFrame::GetMinTimestamp() const {
        return this->_scan->front().timestamp;
    }

    PosTPointCloud::Ptr LiDARFrame::GetScan() const {
        return this->_scan;
    }

    double LiDARFrame::GetTimestamp() const {
        return _timestamp;
    }

    std::ostream &operator<<(std::ostream &os, const LiDARFrame &frame) {
        os << "size: " << frame._scan->size()
           << ", width: " << frame._scan->width
           << ", height: " << frame._scan->height
           << ", timestamp: " << frame._timestamp;
        return os;
    }

    void LiDARFrame::SetTimestamp(double timestamp) {
        _timestamp = timestamp;
    }

    void LiDARFrame::SaveToFile(const std::string &filename) const {
        ns_log::FileLogger logger(filename);
        logger.setPrecision(CalibConfig::CalibData::OutputData::Precision);
        logger.plaintext("timestamp: ", _timestamp);
        for (const auto &p: *_scan) {
            logger.plaintext(p.timestamp, " ", p.x, " ", p.y, " ", p.z);
        }
    }

    bool
    LiDARFrame::SaveFramesToDisk(const std::string &directory, const aligned_vector<LiDARFrame::Ptr> &frames,
                                 int precision) {
        // prepare the directory
        std::string absolutePath = std::filesystem::canonical(directory).c_str();
        std::filesystem::create_directory(absolutePath + "/scans");
        auto digNum = std::to_string(frames.size()).size();

        // prepare the imgPathWithTimeFile
        std::fstream file(absolutePath + "/frames.txt", std::ios::out);
        // header
        file << "# element " + std::to_string(frames.size()) + '\n'
             << "# property double timestamp\n"
             << "# property string path\n"
             << "# end header\n";
        // data and info
        file << std::fixed << std::setprecision(precision);
        for (int i = 0; i != frames.size(); ++i) {
            const auto &frame = frames[i];

            // generate the image name
            std::string curScanName;
            std::stringstream stream2;
            stream2 << std::setfill('0') << std::setw(static_cast<int>(digNum)) << i;
            stream2 >> curScanName;
            curScanName = absolutePath + "/scans/" + curScanName + ".pcd";

            // save date
            pcl::io::savePCDFile(curScanName, *frame->GetScan(), true);
            file << frame->GetTimestamp() << ' ' << curScanName << std::endl;
        }

        file.close();

        return true;
    }

    ns_elic::aligned_vector<std::pair<double, std::string>>
    LiDARFrame::LoadFramesFromDisk(const std::string &directory) {
        std::string absolutePath = std::filesystem::canonical(directory).c_str();

        std::ifstream file(absolutePath + "/frames.txt", std::ios::in);
        ns_elic::aligned_vector<std::pair<double, std::string>> frames;

        std::string strLine;
        {
            std::getline(file, strLine);
            auto num = std::stoi(SplitString(strLine, ' ').back());
            frames.resize(num);
            std::getline(file, strLine);
            std::getline(file, strLine);
            std::getline(file, strLine);
        }
        for (auto &frame: frames) {
            std::getline(file, strLine);
            auto vec = SplitString(strLine, ' ');
            frame.first = std::stod(vec.at(0));
            frame.second = vec.at(1);
        }

        file.close();

        return frames;
    }

}
