#include <spring_detector/certh_detector.hpp>


void CerthDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("CB");

    cv_bridge::CvImagePtr cv_ptr;

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    rgb = cv_ptr->image;
}


bool CerthDetector::sDetect(spring_detector::springDetect::Request &req, spring_detector::springDetect::Response &res)
{
    cv::Mat spring_image;
    cv::Mat mask;

    cv::Size new_size(3696, 2448);
    cv::resize(rgb,spring_image,new_size);

    convex_hull_points = det.getPosition(params, spring_image, mask, 0, lm_cutoff) ;
    det.getPose(rotY, rotZ);

    std::vector<bool> is_cluttered = findEgdes(spring_image, det.spring_roi);

    springs_array.springs.clear();

    for (uint i=0; i<convex_hull_points.size(); i++)
    {
        for (uint j=0; j<convex_hull_points[i].size(); j++)
        {
            p.x = convex_hull_points[i][j].x;
            p.y = convex_hull_points[i][j].y;
            spring_data.points.push_back(p);
        }
        spring_data.phi = rotY[i];
        spring_data.theta = rotZ[i];
        spring_data.is_cluttered = is_cluttered[i];
        springs_array.springs.push_back(spring_data);
        spring_data.points.clear();
    }

    det.clearVectors();

    //------Printing out the results---------//

    std::cout << "Number of springs: " << springs_array.springs.size() << std::endl;

    for (uint m=0; m<springs_array.springs.size(); m++)
    {
        std::cout << "Number of points: " << springs_array.springs[m].points.size() << std::endl;

        for (uint n=0; n<springs_array.springs[m].points.size(); n++)
        {
            cv::Point pt;
            pt.x = springs_array.springs[m].points[n].x;
            pt.y = springs_array.springs[m].points[n].y;

            std::cout << pt << std::endl;
            cv::circle(spring_image, pt, 4, CV_RGB(0, 0, 0));
        }
    }

//    cv::imwrite("/tmp/test.png", rgb);

    //---------------------------------//

    res.spring_msg = springs_array;
    //TODO:Clear the messages!
    return true;
}

std::vector<bool> CerthDetector::findEgdes(cv::Mat &rgb, std::vector<cv::Rect> &roi)
{
    std::vector<bool> is_cluttered;

    for (int i=0; i<roi.size(); i++)
    {
        cv::Rect rect;
        rect.x = roi[i].x - box_offset;
        rect.y = roi[i].y - box_offset;
        rect.width = roi[i].width + 2*box_offset;
        rect.height = roi[i].height + 2*box_offset;

        if ( rect.x < 0 )
            rect.x = 0;
        if ( rect.y < 0 )
            rect.y = 0;
        if ( (rect.x + rect.width) > (rgb.cols - 1) )
            rect.width = (rgb.cols - 1) - rect.x;
        if ( (rect.y + rect.height) > (rgb.rows - 1) )
            rect.height = (rgb.rows - 1) - rect.y;

        cv::Mat spring_image = rgb(rect);

        cv::Mat grad_x, grad_y, grad;

        cv::cvtColor( spring_image, spring_image, CV_BGR2GRAY );
        cv::GaussianBlur( spring_image, spring_image, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
        cv::Sobel( spring_image, grad_x, CV_8U, 1, 0, 3);
        cv::Sobel( spring_image, grad_y, CV_8U, 0, 1, 3);
        cv::addWeighted( grad_x, 0.5, grad_y, 0.5, 0, grad );

        cv::threshold( grad, grad, 50,255, cv::THRESH_BINARY);

        cv::rectangle(grad, cv::Rect( (grad.cols-roi[i].width)/2, (grad.rows-roi[i].height)/2, roi[i].width, roi[i].height), CV_RGB( 0, 0, 0), -1);

        std::cout << cv::countNonZero(grad) << std::endl;
        float score = (float)cv::countNonZero(grad) / (grad.cols * grad.rows);
        std::cout << "Score = " << score << std::endl << std::endl;

        if (score > threshold)
            is_cluttered.push_back(true);
        else
            is_cluttered.push_back(false);

        cvx::util::imwritef(grad, "/tmp/canny%05d.png", i);
    }

    return is_cluttered;
}
