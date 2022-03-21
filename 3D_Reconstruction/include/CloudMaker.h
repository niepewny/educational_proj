#pragma once

#include <fstream>
#include <future>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "PointReconstructor.h"
#include "PointFinder.h"

class CloudMaker
{
public:

    std::vector<PointFinder> pointFinder;
    std::vector<PointReconstructor> pointReconstructor;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;

    //@directory is the path to a directory containing all the required data
    CloudMaker(std::string directory);
    ~CloudMaker();

    void apply();

private:
    int numOfImgs;

    std::vector<std::vector<cv::KeyPoint>> keypoints;
    std::vector <cv::Mat> descriptors;
    std::vector<cv::Mat> points3d;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    //allows user to skip specific data/lines from the .txt file. 
    //@file is the input .txt file
    //@numOfData is the argument deciding how much data should be skipped
    //@line decides if the line or specific data should be skipped (1 for line)
    void skipData(std::ifstream& file, int numOfData, bool line = 0);

    //changes matrix to a vector
    //input matrix should have form of 1xN - 1 chanel, where N is the number of rows.
    template <typename T>
    void mat2vec(cv::Mat& mat, std::vector<T>& vec);

    //receives camera calibration data from .xml file
    void cameraIntristic(std::string path, cv::Mat& K, std::vector<double>& D, int& imgCols, int& imgRows);

    //receives data from .txt file and creates [R|t] matrixes
    void cameraExtrinsics(std::string path, std::vector<cv::Mat>& RT);

    //creates a PointFinder object
    void makeFinder(PointFinder& pointFinder, std::string path, cv::Mat K, std::vector<double> D);

    //exports points from vector of OpenCV matrixes to a pointcloud
    void exportPoints(std::vector<cv::Mat> points3d, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

};