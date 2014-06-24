#include "qr_detector_zbar/qr_detector_zbar.h"

#include <zbar.h>
#include <opencv/highgui.h>

namespace qr_detector_zbar
{

// Create a zbar reader
zbar::ImageScanner scanner;

// Configure the reader
//scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

void getQrCodesWithPose(const rgbd::RGBDImage& rgbd_image, std::map<std::string,geo::Pose3D>& data)
{
    const cv::Mat& rgb_image = rgbd_image.getRGBImage();

    if (rgb_image.rows == 0 || rgb_image.cols == 0) return;

    // Convert to grayscale
    cv::Mat frame_grayscale;
    cv::cvtColor(rgb_image, frame_grayscale, CV_BGR2GRAY);

    // Obtain image data
    int width = frame_grayscale.cols;
    int height = frame_grayscale.rows;
    uchar *raw = (uchar *)(frame_grayscale.data);

    // Wrap image data
    zbar::Image zbar_image(width, height, "Y800", raw, width * height);

    // Scan the image for barcodes
    scanner.scan(zbar_image);

    unsigned int factor = rgb_image.cols / rgbd_image.getDepthImage().cols;

    // Extract results
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {

        if (symbol->get_data() == "") continue;

        // Get the points
        bool valid = false;
        std::vector<cv::Point2i> pts(4);
        std::vector<geo::Vector3> v(4);
        for(int i = 0; i < symbol->get_location_size(); ++i) {
            pts[i] = cv::Point(symbol->get_location_x(i),symbol->get_location_y(i));
            if (!rgbd_image.getPoint3D(pts[i].x / factor, pts[i].y / factor, v[i])) {
                valid = false;
                break;
            } else {
                valid = true;
                v[i].setX(-v[i].x()); //geolib fix
            }
        }

        if (!valid) {
            continue;
        }

        // Set the origin
        geo::Pose3D origin;
        origin.setIdentity();
        origin.setOrigin(v[0]);

        // Calculate the rotation
        geo::Vector3 b3 = (v[1]-v[0]).cross(v[3]-v[0]).normalize();
        geo::Vector3 b1 = b3.cross(v[3]-v[0]).normalize();
        geo::Vector3 b2 = b3.cross(b1);
        tf::Matrix3x3 basis(b1.x(),b1.y(),b1.z(),b2.x(),b2.y(),b2.z(),b3.x(),b3.y(),b3.z());

        // Set the rotation
        geo::Pose3D rotation;
        rotation.setIdentity();
        rotation.setBasis(basis);

//        // DEBUGGING
//        cv::Mat result = rgb_image;
//        cv::circle(result,pts[0],2,cv::Scalar(255,0,0),5);
//        cv::circle(result,pts[1],2,cv::Scalar(0,255,0),5);
//        cv::circle(result,pts[2],2,cv::Scalar(0,0,255),5);
//        cv::circle(result,pts[3],2,cv::Scalar(255,255,0),5);
//        cv::imshow(symbol->get_data(),result);
//        cv::waitKey(3);

        data[symbol->get_data()] = rotation * origin;

    }
}

void getQrCodes(const rgbd::RGBDImage& rgbd_image, std::vector<std::string>& data)
{
    const cv::Mat& rgb_image = rgbd_image.getRGBImage();

    if (rgb_image.rows == 0 || rgb_image.cols == 0) return;

    // Convert to grayscale
    cv::Mat frame_grayscale;
    cv::cvtColor(rgb_image, frame_grayscale, CV_BGR2GRAY);

    // Obtain image data
    int width = frame_grayscale.cols;
    int height = frame_grayscale.rows;
    uchar *raw = (uchar *)(frame_grayscale.data);

    // Wrap image data
    zbar::Image zbar_image(width, height, "Y800", raw, width * height);

    // Scan the image for barcodes
    scanner.scan(zbar_image);

    // Extract results
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
        if (symbol->get_data() == "") continue;
        data.push_back(symbol->get_data());
    }
}


}
