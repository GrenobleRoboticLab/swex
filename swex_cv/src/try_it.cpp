#include "try_it.h"

using namespace sm;

void try_segment();

typedef std::vector<cv::Point>  Polyline;
typedef cv::Vec4i               Line;

cv::Scalar* colors = NULL;

const std::string src_name("Image source.");
const std::string canny_name("Cannyed image.");
const std::string hsv_name("HSV image.");

cv::Mat src_mat;
cv::Mat canny_mat;
cv::Mat hsv_mat;

int     canny_threshold                 = 50;
int     canny_threshold2                = 150;
int     houghLine_threshold             = 80;
int     min_horizontal_length           = 15;
int     horizontal_slope_threshold      = 45;
int     min_vertical_length             = 60;
int     vertical_slope_threshold        = 5;


cv::Scalar random_scalar()
{
    int b = (uchar)random();
    int g = (uchar)random();
    int r = (uchar)random();

    return cv::Scalar(b, g, r);
}

void compute()
{
    std::vector<cv::Vec4i>  lines;
    std::vector<cv::Vec4i>  hor_lines;
    std::vector<cv::Vec4i>  ver_lines;

    double                  tmp_slope;

    cv::Canny(src_mat, canny_mat, canny_threshold, canny_threshold2);
    cv::HoughLinesP(canny_mat, lines, 1, CV_PI / 180, houghLine_threshold+1, 10, 10);

    for (uint i = 0; i < lines.size(); i++)
    {
        double x            = (lines[i][3] - lines[i][1]);
        double y            = (lines[i][2] - lines[i][0]);
        double line_length  = sqrt(pow(x, 2) + pow(y, 2));
        tmp_slope           = abs(180 * (2 * atan(y / (x + line_length))) / CV_PI);

        if (((tmp_slope < (180 + 5) && tmp_slope > (180 - 5)))
          && (line_length > min_vertical_length))
            ver_lines.push_back(lines[i]);
        else if (((tmp_slope < (90 + 45) && tmp_slope > (90 - 45)))
              &&  line_length > min_horizontal_length)
            hor_lines.push_back(lines[i]);
        cv::line(src_mat, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255, 255, 255), 2);
    }


    std::vector<Line>       tmpHorLines;
    std::vector<cv::Point>  intersections;
    cv::Rect                img(0, 0, src_mat.cols, src_mat.rows);
    double                  yVanish = 0.;
    double                  ySum = 0.;
    int                     count = 0;

    for (uint i = 1; i < hor_lines.size(); i++)
    {
        uint j = i-1;
        // on trouve l'equation des droites i et i-1 de la forme : ax + by + c = 0
        int ai  = (hor_lines[i][1] - hor_lines[i][3]),
            bi  = (hor_lines[i][2] - hor_lines[i][0]),
            ci  = ((hor_lines[i][0] * hor_lines[i][3]) - (hor_lines[i][2] * hor_lines[i][1])),
            aj  = (hor_lines[j][1] - hor_lines[j][3]),
            bj  = (hor_lines[j][2] - hor_lines[j][0]),
            cj  = ((hor_lines[j][0] * hor_lines[j][3]) - (hor_lines[j][2] * hor_lines[j][1])),
            wx  = (bj * ci) - (cj * bi),
            wy  = (cj * ai) - (aj * ci),
            w   = (aj * bi) - (bj * ai);
        intersections.push_back(cv::Point(wx / ((w != 0) ? w : 1), wy / ((w != 0) ? w : 1)));
    }

    // on calcul la moyenne des intersections à l'intérieur de l'image
    for (uint i = 0; i < intersections.size(); i ++)
    {
        if (img.contains(intersections[i]))
        {
            ySum += intersections[i].y;
            count ++;
        }
    }

    yVanish = ySum / count;

    for (uint i = 0; i < hor_lines.size(); i++)
        if ((hor_lines[i][1] >= yVanish) && (hor_lines[i][3] >= yVanish)) tmpHorLines.push_back(hor_lines[i]);

    hor_lines = tmpHorLines;

    cv::line(src_mat, cv::Point(0, yVanish), cv::Point(src_mat.cols, yVanish), cv::Scalar(0, 0, 255), 2);


    int yMid = src_mat.rows / 2;
    std::vector<Line> tmpVerLines;
    for (uint i = 0; i < ver_lines.size(); i++)
    {
        if(ver_lines[i][1] > yMid)
            tmpVerLines.push_back(ver_lines[i]);
    }

    ver_lines = tmpVerLines;

    cv::line(src_mat, cv::Point(0, yMid), cv::Point(src_mat.cols, yMid), cv::Scalar(0, 0, 255), 2);

    std::vector<double> m_vBottomScore;
    uint        i       = 0;
    Polyline    poly;
    m_vBottomScore.clear();
    m_vBottomScore.resize(hor_lines.size(), 0);

    std::sort(ver_lines.begin(), ver_lines.end(), lineFirstXComparator);

    if (ver_lines.size())
        poly.push_back(cv::Point(0, ver_lines[0][1]));

    for (; i < ver_lines.size(); i++)
        poly.push_back(cv::Point(ver_lines[i][0], ver_lines[i][1]));

    if(ver_lines.size())
        poly.push_back(cv::Point(src_mat.cols, ver_lines[i][1]));

    for (uint i = 0; i < hor_lines.size(); i++)
    {
        Line        l       = hor_lines[i];
        cv::Vec2i   eq      = getLineEq(l);
        double      weight  = 0;

        for (int x = l[0]; x <= l[2]; x++)
            weight += dPoint2Poly(cv::Point(x, applyStraightEq(x, eq)), poly);
        m_vBottomScore[i] = exp(-(weight / (2 * pow(30, 2))));
    }

    if (poly.size() > 0)
    {
        for (uint i = 1; i < poly.size(); i ++)
            cv::line(src_mat, poly[i-1], poly[i], cv::Scalar(255, 255, 0), 2);
    }

    for (uint j = 0; j < ver_lines.size(); j++)
        cv::line(src_mat, cv::Point(ver_lines[j][0], ver_lines[j][1]), cv::Point(ver_lines[j][2], ver_lines[j][3]), cv::Scalar(255, 0, 0), 2);
    for (uint j = 0; j < hor_lines.size(); j++)
        cv::line(src_mat, cv::Point(hor_lines[j][0], hor_lines[j][1]), cv::Point(hor_lines[j][2], hor_lines[j][3]), cv::Scalar(0, 255, 0), 2);

    cv::cvtColor(src_mat, hsv_mat, CV_BGR2HSV);
}

void show()
{
    cv::imshow(src_name, src_mat);
    cv::imshow(canny_name, canny_mat);
    cv::imshow(hsv_name, hsv_mat);
    cv::waitKey(5);
}

void compute_n_show()
{
    try_segment();
    compute();
    show();
}

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

    if (colors == NULL)
    {
        // pick random colors for each component
        colors = new cv::Scalar[src_mat.cols*src_mat.rows];

        for (int i = 0; i < src_mat.cols*src_mat.rows; i++)
            colors[i] = random_scalar();
    }

    compute_n_show();
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "try_it");

    ros::NodeHandle                 nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber     sub;

    cv::namedWindow(src_name);
    cv::namedWindow(canny_name);
    cv::namedWindow(hsv_name);

    cv::createTrackbar("threshold : ", canny_name, &canny_threshold, 100);
    cv::createTrackbar("threshold2 : ", canny_name, &canny_threshold2, 300);
    cv::createTrackbar("threshold : ", src_name, &houghLine_threshold, 300);

    sub = it.subscribe("/usb_cam/image_raw", 1,  &image_callback);

    ros::spin();

    delete [] colors;
    return 0;
}

void try_segment()
{
    std::vector<cv::Mat>    rgb;
    cv::Mat                 smooth_r;
    cv::Mat                 smooth_g;
    cv::Mat                 smooth_b;
    cv::Mat                 dst_mat     = src_mat.clone();
    int                     num         = 0;
    int                     height      = src_mat.rows;
    int                     width       = src_mat.cols;
    std::vector<edge>       edges(width*height*4);
    universe*               u           = NULL;

    rgb.resize(3);
    cv::split(src_mat, rgb);

    cv::GaussianBlur(rgb[0], smooth_r, cv::Size(25, 25), 2.5); // sigma
    cv::GaussianBlur(rgb[1], smooth_g, cv::Size(25, 25), 2.5);
    cv::GaussianBlur(rgb[2], smooth_b, cv::Size(25, 25), 2.5);


    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (x < width-1) {
                edges[num].a = y * width + x;
                edges[num].b = y * width + (x+1);
                edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y);
                num++;
            }

            if (y < height-1) {
                edges[num].a = y * width + x;
                edges[num].b = (y+1) * width + x;
                edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x, y+1);
                num++;
            }

            if ((x < width-1) && (y < height-1)) {
                edges[num].a = y * width + x;
                edges[num].b = (y+1) * width + (x+1);
                edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y+1);
                num++;
            }

            if ((x < width-1) && (y > 0)) {
                edges[num].a = y * width + x;
                edges[num].b = (y-1) * width + (x+1);
                edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y-1);
                num++;
            }
        }
    }

    u = segment_graph(width*height, num, edges.data(), 5000); // threshold

    // post process small components
    for (int i = 0; i < num; i++) {
        int a = u->find(edges[i].a);
        int b = u->find(edges[i].b);
        if ((a != b) && ((u->size(a) < 100) || (u->size(b) < 100))) // min size
            u->join(a, b);
    }

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int comp = u->find(y * width + x);
            uchar* pU = dst_mat.ptr(y, x);
            *pU = cv::saturate_cast<uchar>(colors[comp][0]);
            *++pU = cv::saturate_cast<uchar>(colors[comp][1]);
            *++pU = cv::saturate_cast<uchar>(colors[comp][2]);
        }
    }

    cv::namedWindow("color");
    cv::imshow("color", dst_mat);
    cv::waitKey(0);
    delete u;
}
