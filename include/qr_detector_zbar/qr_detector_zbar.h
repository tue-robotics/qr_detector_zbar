#ifndef qr_detector_zbar_H_
#define qr_detector_zbar_H_

#include <opencv/cv.h>
#include <geolib/datatypes.h>

namespace qr_detector_zbar {

void getQrCodes(const cv::Mat& rgb_image, std::map<std::string,geo::Pose3D>& data);

}

#endif
