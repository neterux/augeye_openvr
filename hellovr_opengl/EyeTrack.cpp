#include "EyeTrack.h"

EyeTrack::EyeTrack()
    : calibKeyPressed(false)
    , calibrated(false)
    , img_height(0)
    , img_width(0)
    , pressTime(0)
    , refPtGenerated(false)
{
    ptColor = cv::Scalar(255, 255, 255);
}

void EyeTrack::Init(unsigned int w, unsigned int h)
{
    img_width = w;
    img_height = h;
    int pt_distance = 850;
    gazeCalibPts = {
        cv::Point2f(pt_distance, pt_distance),
        cv::Point2f(img_width - pt_distance, pt_distance),
        cv::Point2f(pt_distance, img_height - pt_distance),
        cv::Point2f(img_width - pt_distance, img_height - pt_distance)
    };
}

void EyeTrack::Calibrate()
{
    if (calibKeyPressed)
    {
        if (pressTime == 0)
        {
            pressTime = SDL_GetTicks();

            ptColor = cv::Scalar(0, 241, 255);
            refPtGenerated = false;
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
            pupilPtStack[Eye_Left].push_back(pupilCenterPt[Eye_Left]);
            pupilPtStack[Eye_Right].push_back(pupilCenterPt[Eye_Right]);
        }

        if (SDL_GetTicks() - pressTime > 5000)
        {
            for (int nEye = 0; nEye < 2; nEye++)
            {
                cv::Mat _mean;
                cv::reduce(pupilPtStack[nEye], _mean, 1, CV_REDUCE_AVG);
                pupilCalibPts[nEye].push_back(cv::Point2f(_mean.at<float>(0, 0), _mean.at<float>(0, 1)));

                cv::Mat _var(pupilPtStack[nEye]);
                cv::reduce(_var.mul(_var), _var, 0, CV_REDUCE_SUM);  // SIGMA_x^2
                _var = _var / pupilPtStack[nEye].size() - _mean.mul(_mean);  // 1/N * SIGMA_x^2 - x_mean^2
                //gazeError[nEye].push_back(cv::Point2f(sqrt(_var.at<float>(0, 0)), sqrt(_var.at<float>(0, 1))));
                std::cout << "\nEye" << nEye << std::endl;
                std::cout << "mean: " << _mean << std::endl;
                std::cout << "std:  " << cv::Point2f(sqrt(_var.at<float>(0, 0)), sqrt(_var.at<float>(0, 1))) << std::endl;

                //TODO: 4回分すべてを合わせて標準偏差を求める
                //m_varStack[nEye][0].push_back(cv::Mat(m_pupilPtStack[nEye]).col(0) - _mean.at<float>(0, 0));
                //m_varStack[nEye][1].push_back(cv::Mat(m_pupilPtStack[nEye]).col(1) - _mean.at<float>(0, 1));

                pupilPtStack[nEye].clear();
                pupilPtStack[nEye].shrink_to_fit();
            }

            ptColor = cv::Scalar(255, 255, 255);
            refPtGenerated = false;

            pressTime = 0;
            calibKeyPressed = false;
        }
    }

    if (pupilCalibPts[CALIB].size() < 4)
    {
        if (!refPtGenerated)
        {
            gazePtCalib = cv::Point2f(0, 0);
            gazePtImg = cv::Mat::zeros(cv::Size(img_width, img_height), CV_8UC3);
            cv::circle(gazePtImg, gazeCalibPts[pupilCalibPts[Eye_Left].size()], 7, ptColor, -1, CV_AA);
            refPtGenerated = true;
        }
    }
    else
    {
        std::vector<cv::Vec2f> correctedGazeCalibPts = {
            gazeCalibPts[2],
            gazeCalibPts[3],
            gazeCalibPts[0],
            gazeCalibPts[1]
        };

        for (int nEye = 0; nEye < 2; nEye++)
        {
            matPupilToGaze[nEye] = cv::getPerspectiveTransform(pupilCalibPts[nEye], correctedGazeCalibPts);
            std::cout << "Homography matrix: Eye" << nEye << std::endl;
            std::cout << matPupilToGaze[nEye] << std::endl;

            //for (int ax = 0; ax < 2; ax++)
            //{
            //    cv::Mat _var = varStack[nEye][ax];
            //    std::cout << _var << std::endl;
            //    std::cout << _var.size() << std::endl;
            //    cv::reduce(_var.mul(_var), _var, 0, CV_REDUCE_SUM);  // SIGMA_x^2
            //    _var = _var / _var.size().height;  // 1/N * SIGMA_x^2 - x_mean^2(0)
            //    gazeError[ax].push_back(cv::Point2f(sqrt(_var.at<float>(0, 0)), sqrt(_var.at<float>(0, 1))));
            //    std::cout << gazeError[ax] << std::endl;
            //}

            pupilCalibPts[nEye].clear();
            pupilCalibPts[nEye].shrink_to_fit();
        }

        gazePtImg = cv::Mat::zeros(cv::Size(20, 20), CV_8UC3);

        calibrated = true;
    }
}

void EyeTrack::CalcurateGaze(cv::Mat proj[2], cv::Mat rot, cv::Mat trns)
{
    std::vector<cv::Vec2f> src_R, src_L;
    {
        std::lock_guard<std::mutex> lock(mtx);
        src_L.push_back(pupilCenterPt[Eye_Left]);
        src_R.push_back(pupilCenterPt[Eye_Right]);
    }

    cv::perspectiveTransform(src_L, gazePt[Eye_Left], matPupilToGaze[Eye_Left]);
    cv::perspectiveTransform(src_R, gazePt[Eye_Right], matPupilToGaze[Eye_Right]);

    cv::Mat_<float> p = cv::Mat_<float>::zeros(4, 1);
    cv::triangulatePoints(proj[Eye_Left], proj[Eye_Right], gazePt[Eye_Left], gazePt[Eye_Right], p);
    if (p(3, 0) != 0)
    {
        p /= p(3, 0);
        p = cv::Mat(trns * (cv::Mat_<float>(4, 1) << p(0, 0), p(1, 0), p(2, 0), 1.f));
        p /= p(3, 0);
        gazeDepthPt = (cv::Mat_<float>(3, 1) << p(0, 0), p(1, 0), p(2, 0));

        gazeDepthLen = gazeDepthPt.at<float>(2, 0);
        //gazeDepthLen = cv::Mat(rot * gazeDepthPt).at<float>(2, 0);
        //std::cout << "Depth: " << gazeDepthLen << std::endl;
    }
}

cv::Point3f EyeTrack::GetGazeDepthPt()
{
    return gazeDepthPt;
}

float EyeTrack::GetGazeDepthLen()
{
    return gazeDepthLen;
}
