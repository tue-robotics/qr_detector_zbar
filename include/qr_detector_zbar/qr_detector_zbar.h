#ifndef qr_detector_zbar_H_
#define qr_detector_zbar_H_

#include <opencv2/core.hpp>
#include <rgbd/image.h>

namespace qr_detector_zbar {

void getQrCodes(const cv::Mat& rgb_image, std::map<std::string, std::vector<cv::Point2i> >& data);

}

#endif
