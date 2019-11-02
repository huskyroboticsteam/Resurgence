#include "Tag.h"

namespace AR
{
    double findAngle(cv::Point& a, cv::Point& b, cv::Point& c)
    {
        a = a - b; // shift vectors so that B is at the origin
        b = b - b;
        c = c - b;
        double dot_product = a.ddot(c);
        double a_mag = sqrt((a.x * a.x) + (a.y * a.y));
        double c_mag = sqrt((c.x * c.x) + (c.y * c.y));
        return acos(dot_product / (a_mag * c_mag));
    }

    Tag::Tag(cv::Point top_left, cv::Point top_right, cv::Point bottom_right, cv::Point bottom_left)
    {
        // validate points
        if(top_left.x - top_right.x > 0 ||       // top left and top right are inverted
        top_left.y - bottom_left.y > 0 ||     // top left and bottom left are inverted
        bottom_left.x - bottom_right.x > 0 || // bottom left and bottom right are inverted
        top_right.y - bottom_right.y > 0)     // top right and bottom right are inverted
        {
            throw "Invalid corners (shape crosses itself)";
        }

        // fill vector with points
        std::vector<cv::Point> points;
        points.push_back(top_left);
        points.push_back(top_right);
        points.push_back(bottom_right);
        points.push_back(bottom_left);

        // turn points into Corners
        for(size_t i = 0; i < points.size(); i++)
        {
            size_t last_index = i == 0 ? points.size() - 1 : i - 1;
            size_t next_index = i == points.size() - 1 ? 0 : i + 1;
            double internal_angle = findAngle(points[last_index], points[i], points[next_index]);
            corners.push_back(Corner{internal_angle, points[i]});
        }
    }

    std::vector<Corner> Tag::getCorners() const 
    {
        return corners;
    }

    cv::Vec3d calcOrientation() 
    {
        std::vector<cv::Point> image_points;
        std::vector<cv::Point> object_points;
        for (int i = 0; i < corners.size(); i++)
        {
            image_points.push_back(corners[i]);
        }
        object_points.push_back(cv::Point(2i));
        object_points.push_back(cv::Point(2i));
        object_points.push_back(cv::Point(2i));
        object_points.push_back(cv::Point(2i));
        cv::Mat rvec;
        cv::Mat tvec;
        cv::solvePnP(object_points, image_points, CAMERA_PARAMS, DISTORTION_PARAMS, rvec, tvec);
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);

    }

} // namespace AR
