#pragma once

#include <iostream>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

class PointReconstructor
{
public:

    static bool applyNext;

    PointReconstructor(cv::Mat K, cv::Mat RTl, cv::Mat RTr, int imgCols, int imgRows);

    void apply(std::vector<cv::KeyPoint>& keypointsl, cv::Mat descriptorsl, std::vector<cv::KeyPoint>& keypointsr, cv::Mat& descriptorsr, cv::Mat& points3d, bool initAll);

    //setters:

    static void PointReconstructor::setZmin(float zmin);

    static void PointReconstructor::setZmax(float zmax);

    static void PointReconstructor::setROIsize(int roisize);

    static void PointReconstructor::setMatcherMaxError(int maxerror);

private:
    static float Zmin, Zmax;
    static int ROIsize, maxError, imgCols, imgRows;
    static cv::Mat K;
    static bool init[2];
    cv::Mat RTr2l, RTl, RTr, F;
    std::vector<cv::Mat> projMatr;
    std::vector<std::vector<std::vector<cv::KeyPoint>>>ROIpointsR;
    std::vector<std::vector<cv::Mat>>ROIdesR;

    //splits columns of matrix and stores it in vector of 1-column matrixes
    void mat2vec(cv::Mat& mat, std::vector<cv::Mat>& vec);

    //creates a fundamental matrix 
    //[https://www.sciencedirect.com/topics/engineering/fundamental-matrix]
    void crFundamental();

    //creates a relative [R|t] matrix (right camera coordinates to left camera coordinates)
    void Pr2PlMat();

    //predicts arrays where corresponding points from right camera images may be found on left images depending on Z coordinate, in order 
    // to maximalize effectiveness of the matcher. Though I was unable to adapt [R|t] matrix
    // (problem realted to the one from link: [https://stackoverflow.com/questions/32175286/strange-issue-with-stereo-triangulation-two-valid-solutions]), 
    //the function requires further development
    void createROIs(std::vector<cv::KeyPoint>& keypointsR, cv::Mat& descriptorsR);

    //matches points from left camera images with points from right camera images and pre-filters the matches
    void match(std::vector<cv::KeyPoint>& keypointsL, cv::Mat& descriptorsL,
        std::vector < std::vector < cv::Point_<double>>>& matchedPoints);
   
    void determine3d(std::vector <std::vector < cv::Point_<double>>> matchedPoints, std::vector<cv::Mat>projMatr, cv::Mat& points3d);

    //generates skew-symmetric matrix
    template <typename T>
    cv::Mat skew(cv::Mat V);
};