#include <iostream>
#include <fstream>
#include <vector>
#include <future>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/core/cuda.hpp>


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
        cv::Mat Dmat;

        file.open(path, cv::FileStorage::READ);
        file["Camera_Matrix"] >> K;
        file["Distortion_Coefficients"] >> Dmat;
        file.release();
        Mat2Vec(Dmat, D);
    }

    cv::Mat K;
    std::vector<float> D;

private:

    void Mat2Vec(cv::Mat &mat, std::vector<float> &D)
    {
        for (int i = 0; i < mat.rows; i++)
        {
            D.push_back(mat.at<double>(i, 0));
        }
    }
    
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
            T.push_back(cv::Mat::zeros(3, 1, CV_32F));
            R.push_back(cv::Mat::zeros(3, 3, CV_32F));

            skipData(file, 1);

            for (int j = 0; j < 3; j++)
            {
                file >> T[i].at<float>(j, 0);
            }
            skipData(file, 3);

            for (int j = 0; j < 3; j++)
            {
                for (int k = 0; k < 3; k++)
                {
                    file >> R[i].at<float>(j, k);
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

void preprocess(std::vector<cv::Mat>& img, cv::Mat K, std::vector<float> D, int numOfImgs) {

    //about 15sec, probably worth to make it async
    //for (int i = 0; i < numOfImgs; i++)
    //{
    //    std::async(std::launch::async, cv::undistort, std::ref(img[i]), std::ref(img[i]), std::ref(K), std::ref(D));
    //}

    for (int i = 0; i < numOfImgs; i++) //make it parallel
    {
        cv::undistort(img[i].clone(), img[i], K, D);

        cv::cuda::GpuMat GPUimg(img[i]);
        cv::cuda::cvtColor(GPUimg, GPUimg, cv::COLOR_RGB2GRAY);

        cv::Ptr<cv::cuda::Filter> median = cv::cuda::createMedianFilter(GPUimg.type(), 9);
        median->apply(GPUimg, GPUimg);

        cv::cuda::equalizeHist(GPUimg, GPUimg);

        cv::cuda::GpuMat blur;
        cv::Ptr<cv::cuda::Filter> gaussian = cv::cuda::createGaussianFilter(GPUimg.type(), GPUimg.type(), cv::Size(17, 17), 15.0);
        gaussian->apply(GPUimg, blur);
        cv::cuda::addWeighted(GPUimg, 2.0, blur, -1.0, 0, GPUimg);

        GPUimg.download(img[i]);
    }
}

void cerateROIs(std::vector<std::vector<cv::KeyPoint>> &keypoints, std::vector<cv::Mat> &descriptors,
    std::vector<std::vector<std::vector<cv::KeyPoint>>> &ROIpoints, std::vector<std::vector<cv::Mat>> &ROIdes, int maxYdif)
{
    for (int i = 0; i < keypoints.size(); i++)
    {
        for (int j = 0; j < keypoints[i].size(); j++)
        {
            ROIpoints[i][(int)(keypoints[i][j].pt.y / maxYdif)].push_back(keypoints[i][j]);
            ROIdes[i][keypoints[i][j].pt.y / maxYdif].push_back(descriptors[i].row(j));
            ROIpoints[i][(int)(keypoints[i][j].pt.y / maxYdif + 1)].push_back(keypoints[i][j]);
            ROIdes[i][keypoints[i][j].pt.y / maxYdif + 1].push_back(descriptors[i].row(j));
        }
    }
}

void findPoints(cv::Ptr<cv::ORB> orbPtr, cv::Mat &img, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    orbPtr->detectAndCompute(img, cv::Mat(), keypoints, descriptors);
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

    cv::Ptr<cv::ORB> orbPtr = cv::ORB::create(50000, 1.05f, 4, 15,
        0, 2, cv::ORB::HARRIS_SCORE, 15, 3);

    std::vector<std::vector<cv::KeyPoint>> keypoints(numOfImgs);
    std::vector<cv::Mat> descriptors(numOfImgs);

    for (int i = 0; i < numOfImgs; i++)
    {
        std::async(std::launch::async, findPoints, std::ref(orbPtr), std::ref(img[i]), std::ref(keypoints[i]), std::ref(descriptors[i]));
    }

    int maxYdif = 80;
    int ROIrows = 2 * maxYdif;
    //there exist 2 extra ROIs (index == 0), so that exery point can be stored in two ROIs. In further calculation first index in ROI[index] should be 1, last numOfROIs - 1;
    int numOfROIs = img[0].rows / maxYdif + 1;
    std::vector<std::vector<std::vector<cv::KeyPoint>>> ROIpoints(numOfImgs, std::vector<std::vector<cv::KeyPoint>>(numOfROIs));
    std::vector<std::vector<cv::Mat>> ROIdes(numOfImgs, std::vector<cv::Mat>(numOfROIs));

    cerateROIs(keypoints, descriptors, ROIpoints, ROIdes, maxYdif);

    auto matcherPtr = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
    cv::Mat efect;
    cv::DrawMatchesFlags flags1 = cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
    cv::DrawMatchesFlags flags2 = cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS;

    for (int i = 1; i < numOfROIs - 1; i++)
    {
        std::vector<std::vector<cv::DMatch>> matches;

        if (ROIpoints[0][i].size() > 0 && ROIpoints[1][i].size() > 0)
        {
            cv::cuda::GpuMat des1(ROIdes[0][i]);
            cv::cuda::GpuMat des2(ROIdes[1][i]);
            matcherPtr->radiusMatch(des1, des2, matches, 15);
        }
        //if (i == 1)
        //{
        //    cv::drawMatches(img[0], ROIpoints[0][i], img[1], ROIpoints[1][i], matches, efect,
        //        cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<std::vector<char> >(), flags1);
        //}
        //else
        //{

        //    cv::drawMatches(img[0], ROIpoints[0][i], img[1], ROIpoints[1][i], matches, efect,
        //        cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<std::vector<char> >(), flags2);
        //}
    }

    return 0;
}