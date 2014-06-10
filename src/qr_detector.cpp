#include <ros/ros.h>
#include <rgbd_transport/Client.h>

#include <opencv2/highgui/highgui.hpp>
#include <zbar.h>

#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"sac_plane_seg");

    ros::NodeHandle nh;

    rgbd::Client client;
    client.intialize("/amigo/top_kinect/rgbd");

    // Create a zbar reader
    zbar::ImageScanner scanner;

    // Configure the reader
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    std::cout << "init" << std::endl;

    ros::Rate r(30);
    while (ros::ok())
    {
        r.sleep();

        std::cout << "loop" << std::endl;

        //! Only perform hook if a new image is available
        rgbd::RGBDImageConstPtr image = client.nextImage();
        if (!image)
            continue;

        const cv::Mat& rgb_image = image->getRGBImage();

        cv::Mat frame_grayscale;

        // Convert to grayscale
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
        int counter = 0;
        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {



            // do something useful with results
            std::cout    << "decoded " << symbol->get_type_name()
                    << " symbol \"" << symbol->get_data() << '"' <<std::endl;

//            //cout << "Location: (" << symbol->get_location_x(0) << "," << symbol->get_location_y(0) << ")" << endl;
//            //cout << "Size: " << symbol->get_location_size() << endl;

//            // Draw location of the symbols found
//            if (symbol->get_location_size() == 4) {
//                //rectangle(frame, Rect(symbol->get_location_x(i), symbol->get_location_y(i), 10, 10), Scalar(0, 255, 0));
//                line(frame, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 255, 0), 2, 8, 0);
//                line(frame, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0, 255, 0), 2, 8, 0);
//                line(frame, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0, 255, 0), 2, 8, 0);
//                line(frame, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0, 255, 0), 2, 8, 0);
//            }

//            // Get points
//            /*for (Symbol::PointIterator point = symbol.point_begin(); point != symbol.point_end(); ++point) {
//                cout << point << endl;
//            } */
            counter++;
        }

        // Show captured frame, now with overlays!
        imshow("captured", rgb_image);

        // clean up
//        image.set_data(NULL, 0);

        cv::waitKey(30);

        std::cout << "Loop took " << r.cycleTime() << std::endl;
    }
}
