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

void Mat2Vec(cv::Mat& mat, std::vector<float>& vec)
{
    for (int i = 0; i < mat.rows; i++)
    {
        vec.push_back(mat.at<double>(i, 0));
    }
}

void cameraIntristic(std::string path, cv::Mat& K, std::vector<float>& D)
{
    cv::FileStorage file;
    cv::Mat Dmat;

    file.open(path, cv::FileStorage::READ);
    file["Camera_Matrix"] >> K;
    file["Distortion_Coefficients"] >> Dmat;
    file.release();
    Mat2Vec(Dmat, D);
}

void cameraExtrinsics(std::string path, std::vector<cv::Mat> &RT)
{
    std::ifstream file;
    file.open(path);

    if (!file)
    {
        std::cout << "Bad extrinsics .txt file" << std::endl;
        exit(1);
    }

    skipData(file, 2, 1);

    for (int i = 0; i < RT.size(); i++)
    {
        RT[i] = cv::Mat::zeros(3, 4, CV_32F);

        skipData(file, 1);

        for (int j = 0; j < 3; j++)
        {
            file >> RT[i].at<float>(j, 3);
        }
        skipData(file, 3);

        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                file >> RT[i].at<float>(j, k);
            }
        }
    }

    file.close();
}

void preprocess(cv::Mat& img, cv::Mat &K, std::vector<float> &D) 
{
    cv::undistort(img.clone(), img, K, D);

    cv::cuda::GpuMat GPUimg(img);
    cv::cuda::cvtColor(GPUimg, GPUimg, cv::COLOR_RGB2GRAY);

    ////cv::cuda::equalizeHist(GPUimg, GPUimg);
    //cv::Ptr<cv::cuda::CLAHE> clahe = cv::cuda::createCLAHE();
    //clahe->setClipLimit(15);
    //clahe->apply(GPUimg, GPUimg);


    //cv::Ptr<cv::cuda::Filter> median = cv::cuda::createMedianFilter(GPUimg.type(), 10);
    //median->apply(GPUimg, GPUimg);

    //cv::cuda::threshold(GPUimg, GPUimg, 150, 255, cv::THRESH_BINARY);

    //cv::cuda::GpuMat blur;
    //cv::Ptr<cv::cuda::Filter> gaussian = cv::cuda::createGaussianFilter(GPUimg.type(), GPUimg.type(), cv::Size(17, 17), 15.0);
    //gaussian->apply(GPUimg, blur);
    //cv::cuda::addWeighted(GPUimg, 2.0, blur, -1.0, 0, GPUimg);

    GPUimg.download(img);
}

void cerateROIs(std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, int maxYdif,
    std::vector<std::vector<cv::KeyPoint>> &ROIpoints, std::vector<cv::Mat> &ROIdes)
{
    for (int i = 0; i < keypoints.size(); i++)
    {
        ROIpoints[(int)(keypoints[i].pt.y / maxYdif)].push_back(keypoints[i]);
        ROIdes[keypoints[i].pt.y / maxYdif].push_back(descriptors.row(i));
        ROIpoints[(int)(keypoints[i].pt.y / maxYdif + 1)].push_back(keypoints[i]);
        ROIdes[keypoints[i].pt.y / maxYdif + 1].push_back(descriptors.row(i));
    }
}

void findPoints
    (
    std::string path, cv::Mat& img,
    cv::Mat &K, std::vector<float> &D, 
    cv::Ptr<cv::ORB> orbPtr, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, 
    int maxYdif, std::vector<std::vector<cv::KeyPoint>> &ROIpoints, std::vector<cv::Mat> &ROIdes
    )
{
    img = cv::imread(path);

    if (img.size == 0)
    {
        std::cout << "Image reading error :(" << std::endl;
        exit(0);
    }

    preprocess(img, K, D);
    orbPtr->detectAndCompute(img, cv::Mat(), keypoints, descriptors);
    cerateROIs(keypoints, descriptors, maxYdif, ROIpoints, ROIdes);
}


int main()
{

    std::string directory = "C:/Users/Wojtas/Downloads/data-20211203T205153Z-001/data";

    cv::Mat K;
    std::vector<float> D;
    cameraIntristic(directory + "/camera_calibration.xml", K, D);

    std::vector<std::string> fn;
    cv::glob(directory + "/*.JPG", fn, false);
    int numOfImgs = fn.size();

    std::vector<cv::Mat> RT(numOfImgs);
    cameraExtrinsics(directory + "/exterior_orientation.txt", RT);

    std::vector<cv::Mat> img(numOfImgs);

    cv::Ptr<cv::ORB> orbPtr = cv::ORB::create(1000000, 1.05f, 2, 20,
        0, 2, cv::ORB::HARRIS_SCORE, 20, 1);

    std::vector<std::vector<cv::KeyPoint>> keypoints(numOfImgs);
    std::vector<cv::Mat> descriptors(numOfImgs);

    int maxYdif = 60;
    int ROIrows = 2 * maxYdif;
    int imgRows = 3000; //temporary. To take from .xml
    //there exist 2 extra ROIs, so that exery point can be stored in two ROIs. In further calculation (apart from creating), first index in ROI[index] should be 1, last numOfROIs - 1;
    int numOfROIs = imgRows / maxYdif + 1;
    std::vector<std::vector<std::vector<cv::KeyPoint>>> ROIpoints(numOfImgs, std::vector<std::vector<cv::KeyPoint>>(numOfROIs));
    std::vector<std::vector<cv::Mat>> ROIdes(numOfImgs, std::vector<cv::Mat>(numOfROIs));

    std::vector<std::thread> T;
    for (int i = 0; i < numOfImgs; i++) 
    {
        T.push_back(std::thread(findPoints, fn[i], std::ref(img[i]), std::ref(K), std::ref(D), std::ref(orbPtr), std::ref(keypoints[i]), std::ref(descriptors[i]), maxYdif, std::ref(ROIpoints[i]), std::ref(ROIdes[i])));
    }
    for (int i = 0; i < numOfImgs; i++)
    {
        T[i].join();
    }
    cv::Ptr< cv::cuda::DescriptorMatcher> matcherPtr = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);

    std::vector<cv::DMatch> semiFilterredMatches;
    std::vector < std::vector<cv::KeyPoint>> matchedPoints(2);


    for (int i = 1; i < numOfROIs-1; i++)
    {
        std::vector<std::vector<cv::DMatch>> matches;

        if (ROIpoints[0][i].size() > 0 && ROIpoints[1][i].size() > 0)
        {
            cv::cuda::GpuMat des1(ROIdes[1][i]);
            cv::cuda::GpuMat des2(ROIdes[2][i]);
            
            //mask would be too memory-consuming
            matcherPtr->knnMatch(des1, des2, matches, 2);
            for (int j = 0; j < matches.size(); j++)
            {
                if (matches[j][0].distance < 0.90 * matches[j][1].distance)
                {
                    matches[j][0].imgIdx = i;
                    semiFilterredMatches.push_back(matches[j][0]);
                }
            }
        }
    }
    std::sort(semiFilterredMatches.begin(), semiFilterredMatches.end(), [semiFilterredMatches](cv::DMatch match1, cv::DMatch match2)
        {
            return(match1.distance < match2.distance);
        });
    ///
    cv::Mat A = img[1];
    cv::Mat B = img[2];
    std::vector<cv::Scalar>color{ cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0),  cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 255) };
    for (int j = 0; j < 4; j++)
    {
        for (int i = semiFilterredMatches.size() * 0.05 * j; i < semiFilterredMatches.size() * 0.05 * (j + 1); i++)
        {
            matchedPoints[0].push_back(ROIpoints[1][semiFilterredMatches[i].imgIdx][semiFilterredMatches[i].queryIdx]);
            matchedPoints[1].push_back(ROIpoints[2][semiFilterredMatches[i].imgIdx][semiFilterredMatches[i].trainIdx]);
        }
        cv::drawKeypoints(A, matchedPoints[0], A, color[j]);
        cv::drawKeypoints(B, matchedPoints[1], B, color[j]);

        matchedPoints[0].clear();
        matchedPoints[1].clear();
    }

    return 0;
}