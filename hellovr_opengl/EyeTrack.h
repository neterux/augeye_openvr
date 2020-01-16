#pragma once
#include <mutex>
#include <opencv2/opencv.hpp>
#include "pupil.h"
#include <SDL.h>

class EyeTrack
{
public:
    EyeTrack();
    void Init(unsigned int w, unsigned int h);
    //~EyeTrack();

    bool calibrated;
    bool calibKeyPressed;

    std::mutex mtx;

    cv::Vec2f pupilCenterPt[2];
    //struct pupilPt
    //{
    //    float x;
    //    float y;
    //};
    //std::atomic<pupilPt> pupilCenterPt[2];

    cv::Point2f gazePtCalib;
    std::vector<cv::Vec2f> gazePt[2];
    cv::Vec3f gazeDepthPt;
    float gazeDepthLen[2];
    cv::Mat gazePtImg;  // reference point image variable for calibration

    std::vector<cv::Point2f> gazeError[2];

private:
    const int Eye_Left = 0;
    const int Eye_Right = 1;

    unsigned int img_width, img_height;
    cv::Scalar ptColor;
    std::vector<cv::Point2f> gazeCalibPts;
    bool refPtGenerated;

    const int CALIB = 0;
    unsigned int pressTime;  // msec
    std::vector<cv::Point2f> pupilCalibPts[2];
    std::vector<cv::Point2f> pupilPtStack[2];

    cv::Mat matPupilToGaze[2];  // transform matrix

    cv::Mat varStack[2][2];

public:
    void Calibrate();
    void CalcurateGaze();
    cv::Point3f GetGazeDepth(cv::Mat proj[2], cv::Mat extr[2]);
};
