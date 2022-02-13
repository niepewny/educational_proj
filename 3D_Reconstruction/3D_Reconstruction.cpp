#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>

void skipData(std::ifstream& file, int numOfData, bool line = 0)
{
    std::string skipper;

    if (line)
    {
        for (int i = 0; i < numOfData; ++i) {
            getline(file, skipper, '\n');
        }
    }
    else
    {
        for (int i = 0; i < numOfData; i++)
        {
            file >> skipper;
        }
    }
}

struct CameraIntristic
{
    CameraIntristic(std::string path)
    {
        cv::FileStorage file;
        file.open(path, cv::FileStorage::READ);
        file["Camera_Matrix"] >> K;
        file["Distortion_Coefficients"] >> D;
        file.release();
    }

    cv::Mat K, D;
};

struct CameraExtrinsics
{
    CameraExtrinsics(std::string path, int numOfImgs)
    {
        std::ifstream file;
        file.open(path);

        if (!file)
        {
            std::cout << "Bad extrinsics .txt file" << std::endl;
            exit(1);
        }

        skipData(file, 2, 1);

        for (int i = 0; i < numOfImgs; i++)
        {
            T.push_back(cv::Mat::zeros(3, 1, CV_64F));
            R.push_back(cv::Mat::zeros(3, 3, CV_64F));

            skipData(file, 1);

            for (int j = 0; j < 3; j++)
            {
                file >> T[i].at<double>(j, 0);
            }
            skipData(file, 3);

            for (int j = 0; j < 3; j++)
            {
                for (int k = 0; k < 3; k++)
                {
                    file >> R[i].at<double>(j, k);
                }
            }
        }

        file.close();
    }
    std::vector<cv::Mat> R, T;
};

void readImages(std::string path, std::vector<cv::Mat>& img, int& numOfImgs)
{
    std::vector<std::string> fn;
    cv::glob(path, fn, false);
    numOfImgs = fn.size();

    for (int i = 0; i < numOfImgs; i++)
    {
        img.push_back(cv::imread(fn[i]));
    }
}

void preprocess(std::vector<cv::Mat>& img, cv::Mat K, cv::Mat D, int numOfImgs) {

    for (int i = 0; i < numOfImgs; i++) //make it parallel
    {
        cv::cuda::GpuMat GPUimg(img[i]);

        cv::cuda::cvtColor(GPUimg, GPUimg, cv::COLOR_RGB2GRAY);

        cv::Ptr<cv::cuda::Filter> median = cv::cuda::createMedianFilter(GPUimg.type(), 9);
        median->apply(GPUimg, GPUimg);

        cv::cuda::equalizeHist(GPUimg, GPUimg);


        cv::cuda::GpuMat blur;
        cv::Ptr<cv::cuda::Filter> gaussian = cv::cuda::createGaussianFilter(GPUimg.type(), GPUimg.type(), cv::Size(17, 17), 15.0);
        gaussian->apply(GPUimg, blur);

        cv::cuda::addWeighted(GPUimg, 2.0, blur, -1.0, 0, GPUimg);
        cv::Mat image(GPUimg);
        img[i] = image;
    }
}


int main()
{

    std::string directory = "C:/Users/Wojtas/Downloads/data-20211203T205153Z-001/data";

    CameraIntristic cameraIntristic(directory + "/camera_calibration.xml");

    std::vector<cv::Mat> img;
    int numOfImgs;
    readImages(directory + "/*.JPG", img, numOfImgs);

    CameraExtrinsics CameraExtrinsics(directory + "/exterior_orientation.txt", numOfImgs);

    preprocess(img, cameraIntristic.K, cameraIntristic.D, numOfImgs);

    std::vector<cv::cuda::GpuMat> GPUimg(numOfImgs);
    for (int i = 0; i < numOfImgs; i++)
    {
        GPUimg[i].upload(img[i]);
    }


    cv::Ptr<cv::cuda::ORB> orbPtr = cv::cuda::ORB::create(500000, 1.05f, 4, 31,
        0, 2, cv::ORB::HARRIS_SCORE, 31, 1);

    int ROIrows = 100;
    int numOfROIs = GPUimg[0].rows / ROIrows;
    std::vector<std::vector<cv::KeyPoint>> keypoints(numOfImgs);
    std::vector<cv::cuda::GpuMat> descriptors(numOfImgs);


    for (int i = 0; i < numOfImgs; i++)
    {
        orbPtr->detectAndCompute(GPUimg[i], cv::cuda::GpuMat(), keypoints[i], descriptors[i]);
    }

    std::vector<std::vector<std::vector<cv::KeyPoint>>> ROIpoints(numOfImgs, std::vector<std::vector<cv::KeyPoint>>(numOfROIs));
    std::vector<std::vector<cv::cuda::GpuMat>> ROIdes(ROIrows);

    for (int i = 0; i < numOfImgs; i++)
    {
        for (int j = 0; j < keypoints[i].size(); j++)
        {
            ROIpoints[i][(int)(keypoints[i][j].pt.y / ROIrows)].push_back(keypoints[i][j]);
            ROIdes[keypoints[i][j].pt.y / ROIrows].push_back(descriptors[i].row(i));
        }
    }

    auto matcherPtr = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
    cv::Mat efect;
    cv::DrawMatchesFlags flags = cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS;
    for (int i = 0; i < numOfROIs; i++)
    {
        std::vector<std::vector<cv::DMatch>> matches;

        matcherPtr->radiusMatch(ROIdes[0][i], ROIdes[1][i], matches, 30);

        if (i == 0)
        {
            cv::drawMatches(img[0], ROIpoints[0][i], img[1], ROIpoints[1][i], matches, efect,
                cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<std::vector<char> >(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        }
        else
        {

            cv::drawMatches(img[0], ROIpoints[0][i], img[1], ROIpoints[1][i], matches, efect,
                cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<std::vector<char> >(), flags);
        }

    }


    return 0;
}