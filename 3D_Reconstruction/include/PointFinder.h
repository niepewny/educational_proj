#pragma once

#include <iostream>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>

class PointFinder
{

public:
    static bool applyNext;

    PointFinder();

    PointFinder(std::string path, cv::Mat& k, std::vector<double>& d);

    void apply(std::vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptors);

    //setters:

    static void setGaussianSize(int size);

    static void setGaussianSigma(double sigma);

    static void setORBscaleFactor(float scaleFactor);

    static void setORBnLevels(int nLevels);

    static void setORBedgeThreshold(int threshold);

    static void setORBpatchSize(int size);

    static void setORBfastThreshold(int threshold);

    static void setORBfirstLevel(int firstLevel);

private:
    cv::Mat imgOrg;
    cv::Mat img;
    static cv::Ptr<cv::ORB> ORB;
    static cv::Mat K;
    static std::vector<double> D;
    static int gaussianSize;
    static double gaussianSigma;
    static bool init[2];

    void preprocess();
};