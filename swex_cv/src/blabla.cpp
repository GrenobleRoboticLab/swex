#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <swex_cv/FloorSegmentation.h>

const std::string src_name("name");
cv::Mat src_mat;
FloorSegmentation fs;

void image_callback(const sensor_msgs::ImageConstPtr &imagePtr)
{
    cv_bridge::CvImage      cvBImage;
    cv_bridge::CvImagePtr   cvBImagePtr;

    try
    {
        cvBImagePtr         = cv_bridge::toCvCopy(imagePtr, sensor_msgs::image_encodings::BGR8);
        cvBImage.image      = cvBImagePtr->image;
        cvBImage.encoding   = cvBImagePtr->encoding;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    src_mat = cvBImage.image;
    Polygon out;
    fs.findFloor(src_mat, out);
    //cs.segmentColor(src_mat, out);
/*
    std::cout << out.size() << std::endl;
    for (std::vector<Polygon>::iterator it = out.begin(); it != out.end(); ++it)
    {
        for(uint i = 1; i < (*it).size(); i++)
        {
            std::cout << "[" << (*it)[i].x << " : " << (*it)[i].y << "]" << std::endl;
            cv::line(src_mat, (*it).at(i-1), (*it).at(i), cv::Scalar(255, 0, 0), 2);
        }
    }

*/
    cv::imshow(src_name, src_mat);
    cv::waitKey(5);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "try_it");

    ros::NodeHandle                 nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber     sub;

    cv::namedWindow(src_name);

    sub = it.subscribe("/usb_cam/image_raw", 1,  &image_callback);

    ros::spin();

    return 0;
}
