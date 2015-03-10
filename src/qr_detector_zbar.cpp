#include "qr_detector_zbar/qr_detector_zbar.h"

#include <zbar.h>
#include <opencv/highgui.h>

namespace qr_detector_zbar
{

void getQrCodes(const cv::Mat& rgb_image, std::map<std::string, std::vector<cv::Point2i> >& data)
{
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

    // Create a zbar reader
    zbar::ImageScanner scanner;

    // Configure the reader
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // Scan the image for barcodes
    scanner.scan(zbar_image);

    // Extract results
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
        if (symbol->get_data() == "") continue;
        if (symbol->get_location_size() != 4) continue;

        std::vector<cv::Point2i> pts(4);
        for(int i = 0; i < symbol->get_location_size(); ++i) {
            pts[i] = cv::Point2i(symbol->get_location_x(i),symbol->get_location_y(i));
        }

        data[symbol->get_data()] = pts;
    }
}


}
