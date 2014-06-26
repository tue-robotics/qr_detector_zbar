#ifndef qr_detector_zbar_H_
#define qr_detector_zbar_H_

#include <opencv/cv.h>
#include <geolib/datatypes.h>
#include <rgbd_transport/RGBDImage.h>

namespace qr_detector_zbar {

void getQrCodes(const cv::Mat& rgb_image, std::map<std::string, std::vector<cv::Point2i> >& data);

bool get3DCornerPoints(const cv::Mat& depth_image, const geo::DepthCamera& rasterizer, const std::vector<cv::Point2i>& pts, std::vector<geo::Vector3>& v, unsigned int scale = 1);
bool getPoseFromCornerPoints(const cv::Mat& depth_image, const geo::DepthCamera& rasterizer, const std::vector<cv::Point2i>& pts, geo::Pose3D& pose, unsigned int scale = 1);

}

#endif
