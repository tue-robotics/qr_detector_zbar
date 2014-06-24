#ifndef qr_detector_zbar_H_
#define qr_detector_zbar_H_

#include <opencv/cv.h>
#include <geolib/datatypes.h>
#include <rgbd_transport/RGBDImage.h>

namespace qr_detector_zbar {

void getQrCodesWithPose(const rgbd::RGBDImage& rgbd_image, std::map<std::string,geo::Pose3D>& data);
void getQrCodes(const rgbd::RGBDImage& rgbd_image, std::vector<std::string>& data);

}

#endif
