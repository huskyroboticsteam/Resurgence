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

	cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R)
	{
		double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0)
						+  R.at<double>(1,0) * R.at<double>(1,0));
 
		bool singular = sy < 1e-6;
 
		double x, y, z;
		if (!singular)
		{
			x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
			y = atan2(-R.at<double>(2,0), sy);
			z = atan2(R.at<double>(1,0), R.at<double>(0,0));
		}
		else
		{
			x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
			y = atan2(-R.at<double>(2,0), sy);
			z = 0;
		}
		return cv::Vec3f(x, y, z);    
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

    cv::Vec3d Tag::calcOrientation() const
    {
        std::vector<cv::Point> image_points;
        std::vector<cv::Point> object_points;
        for (int i = 0; i < corners.size(); i++)
        {
            image_points.push_back(corners[i].point);
        }
        object_points.push_back(cv::Point2i(0, 0));
        object_points.push_back(cv::Point2i(200, 0));
        object_points.push_back(cv::Point2i(200, 200));
        object_points.push_back(cv::Point2i(0, 200));
        cv::Mat rvec;
        cv::Mat tvec;
        cv::solvePnP(object_points, image_points, CAMERA_PARAMS, DISTORTION_PARAMS, rvec, tvec);
        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);

		return rotationMatrixToEulerAngles(rmat);
    }

    float Tag::getPitch() const 
    {
        
        if (orientation == NULL) 
        {

            orientation = calcOrientation();

        }

        return orientation[0];

    }

    float Tag::getYaw() const 
    {
        
        if (orientation == NULL) 
        {

            orientation = calcOrientation();

        }

        return orientation[1];

    }
    
    float Tag::getPitch() const 
    {
        
        if (orientation == NULL) 
        {

            orientation = calcOrientation();

        }

        return orientation[2];

    }


} // namespace AR
