#include <ros/ros.h>
#include <rgbd_transport/Client.h>

#include <opencv2/highgui/highgui.hpp>
#include <zbar.h>

#include <iostream>

#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"sac_plane_seg");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("qr_data_topic",0,false);
    rgbd::Client client;
    client.intialize("rgbd_topic");

    // Takes some time to get the first image
    ros::Duration(1.0).sleep();

    ros::Rate r_init(.5);
    while (!client.nextImage() && ros::ok()) {
        ROS_WARN("[QR Detector] : No RGBD image available, is the rgbd server running?");
        r_init.sleep();
    }

    ROS_INFO("[QR Detector] : RGBD Server is running; we are getting images - Throw me some QR Codes :)");

    // Create a zbar reader
    zbar::ImageScanner scanner;

    // Configure the reader
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    ros::Rate r(30);
    while (ros::ok())
    {
        r.sleep();

        //! Only perform hook if a new image is available
        rgbd::RGBDImageConstPtr image = client.nextImage();
        if (!image) {
            ROS_DEBUG("[QR Detector] : NO RGBD Image available..");
            continue;
        }

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
        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
            std_msgs::String s;
            s.data = symbol->get_data();
            pub.publish(s);

            ROS_INFO_STREAM("[QR Detector] : Found marker with data: '\e[101m" << s.data << "\e[0m'");
        }
    }
}
