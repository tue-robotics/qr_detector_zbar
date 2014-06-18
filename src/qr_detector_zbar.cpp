#include "qr_detector_zbar/qr_detector_zbar.h"

#include <zbar.h>

namespace qr_detector_zbar
{

void getQrCodes(const cv::Mat& rgb_image, std::map<std::string,geo::Pose3D>& data)
{
    // Create a zbar reader
    zbar::ImageScanner scanner;

    // Configure the reader
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

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
        data[symbol->get_data()] = geo::Pose3D(0,0,0);

        for (zbar::Symbol::PointIterator point = symbol->point_begin(); point != symbol->point_end(); ++point) {
//            std::cout << (*point).x << std::endl;
        }
    }
}

}
