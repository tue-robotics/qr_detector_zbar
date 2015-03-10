#include "qr_detector_zbar/qr_detector_zbar.h"

#include <ros/ros.h>
#include <rgbd/Client.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"sac_plane_seg");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("qr_data_topic",0,false);
    rgbd::Client client;
    client.intialize("rgbd");

    // Takes some time to get the first image
    ros::Duration(1.0).sleep();

    ros::Rate r_init(.5);
    while (!client.nextImage() && ros::ok()) {
        ROS_WARN("[QR Detector] : No RGBD image available, is the rgbd server running?");
        r_init.sleep();
    }

    ROS_INFO("[QR Detector] : RGBD Server is running; we are getting images - Throw me some QR Codes :)");

    ros::Rate r(30);
    while (ros::ok())
    {
        r.sleep();

        //! Only perform hook if a new image is available
        rgbd::ImageConstPtr image = client.nextImage();
        if (!image) {
            ROS_DEBUG("[QR Detector] : NO RGBD Image available..");
            continue;
        }

        std::map<std::string, std::vector<cv::Point2i> > data;
        qr_detector_zbar::getQrCodes(image->getRGBImage(),data);

        // Extract results
        for (std::map<std::string, std::vector<cv::Point2i> >::const_iterator it = data.begin(); it != data.end(); ++it) {
            std_msgs::String s;
            s.data = it->first;

            pub.publish(s);

            ROS_INFO_STREAM("[QR Detector] : Found marker with data: '\e[101m" << s.data << "\e[0m'");
        }
    }
}
