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
#include <opencv2/xfeatures2d.hpp>

template <typename T>
cv::Mat skew(cv::Mat V)
{
    cv::Mat Vx = (cv::Mat_<T>(3, 3) <<
        0, -V.at<T>(2), V.at<T>(1),
        V.at<T>(2), 0, -V.at<T>(0),
        -V.at<T>(1), V.at<T>(0), 0);
    return Vx;
}

void crFundamental(cv::Mat RTl, cv::Mat RTr, cv::Mat Korg, cv::Mat &F)
{
    cv::Mat relativeRT, R, T, Tx, K;
    cv::Mat hRow = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
    cv::Mat hRTl, hRTr;
    
    vconcat(RTl, hRow, hRTl);
    vconcat(RTr, hRow, hRTr);

    K = Korg.clone();
    K = K.inv();

    relativeRT = hRTr * hRTl.inv();
    R = relativeRT(cv::Rect(0, 0, 3, 3));
    T = relativeRT.col(3);
    Tx = skew<float>(T);
    F.push_back(K.t() * Tx * R * K);

}

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

void mat2vec(cv::Mat& mat, std::vector<cv::Mat>& vec)
{
    for (int i = 0; i < mat.cols; i++)
    {
        vec.push_back(mat.col(i).clone());
    }
}

void mat2vec(cv::Mat& mat, std::vector<float>& vec)
{
    for (int i = 0; i < mat.rows; i++)
    {
        vec.push_back(mat.at<double>(i, 0));
    }
}

void cameraIntristic(std::string path, cv::Mat &K, std::vector<float>& D, int& imgRows)
{
    cv::FileStorage file;
    cv::Mat Dmat;

    file.open(path, cv::FileStorage::READ);
    file["Camera_Matrix"] >> K;
    K.convertTo(K, CV_32F);
    file["Distortion_Coefficients"] >> Dmat;
    file["image_Height"] >> imgRows;
    file.release();
    mat2vec(Dmat, D);
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

    //cv::cuda::equalizeHist(GPUimg, GPUimg);
    cv::Ptr<cv::cuda::CLAHE> clahe = cv::cuda::createCLAHE();
    clahe->setClipLimit(100);
    clahe->apply(GPUimg, GPUimg);

    //cv::Ptr<cv::cuda::Filter> median = cv::cuda::createMedianFilter(GPUimg.type(), 10);
    //median->apply(GPUimg, GPUimg);

    cv::Ptr<cv::cuda::Filter> gaussian = cv::cuda::createGaussianFilter(GPUimg.type(), GPUimg.type(), cv::Size(9, 9), 5);
    gaussian->apply(GPUimg, GPUimg);

    //cv::cuda::GpuMat blur;
    //cv::Ptr<cv::cuda::Filter> gaussian = cv::cuda::createGaussianFilter(GPUimg.type(), GPUimg.type(), cv::Size(21, 21), 15.0);
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

void match(std::vector<std::vector<cv::KeyPoint>> &ROIpointsL, std::vector<cv::Mat> &ROIdesL, std::vector<std::vector<cv::KeyPoint>>& ROIpointsP, std::vector<cv::Mat>& ROIdesP, std::vector < std::vector < cv::Point_<float>>> &matchedPoints)
{
    cv::Ptr< cv::cuda::DescriptorMatcher> matcherPtr = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> matches;
    cv::cuda::GpuMat des1;
    cv::cuda::GpuMat des2;

    for (int roiId = 1; roiId < ROIpointsL.size() - 1; roiId++)
    {
        if (ROIpointsL[roiId].size() > 1 && ROIpointsP[roiId].size() > 1)
        {
            des1.upload(ROIdesL[roiId]);
            des2.upload(ROIdesP[roiId]);
            matcherPtr->knnMatch(des1, des2, matches, 2);

            for (int matchId = 0; matchId < matches.size(); matchId++)
            {
                if ((matches[matchId][0].distance < matches[matchId][1].distance * 0.8 && matches[matchId][0].distance < 4))
                {
                    //in case point needed further filtering
                    //matches[j][0].imgIdx = i;
                    //semiFilterredMatches.push_back(matches[j][0]);
                    matchedPoints[0].push_back(ROIpointsL[roiId][matches[matchId][0].queryIdx].pt);
                    matchedPoints[1].push_back(ROIpointsP[roiId][matches[matchId][0].trainIdx].pt);
                }
            }
        }
    }
}

void determine3d(std::vector <std::vector < cv::Point_<float>>> matchedPoints, std::vector<cv::Mat>projMatr, cv::Mat points3d)
{
    cv::Mat hPoints(4, matchedPoints[0].size(), CV_32FC2);

    cv::triangulatePoints(projMatr[0], projMatr[1], matchedPoints[0], matchedPoints[1], hPoints);

    std::vector<cv::Mat>converter;
    hPoints = hPoints.t();
    mat2vec(hPoints, converter);
    cv::merge(converter, hPoints);

    cv::convertPointsFromHomogeneous(hPoints, points3d);
}

void findPoints
(
    std::string path,
    cv::Mat K, std::vector<float>& D,
    int maxYdif, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors
)
{
    cv::Ptr<cv::ORB> ORB = cv::ORB::create(1000000, 1.1f, 50, 5, //poszukaæ, czy lepiej 1 wspólny, czy 1/thread
        0, 2, cv::ORB::HARRIS_SCORE, 5, 15);
    cv::Mat img;

    img = cv::imread(path);

    if (img.size == 0)
    {
        std::cout << "Image reading error :(" << std::endl;
        exit(0);
    }

    preprocess(img, K, D);
    ORB->detect(img, keypoints);

    for (int i = 0; i < keypoints.size(); i++)
    {
        keypoints[i].angle = 0;
    }

    ORB->compute(img, keypoints, descriptors);

}

void reconstructPoints(std::vector<cv::KeyPoint> &keypointsL, cv::Mat &descriptorsL, std::vector<cv::KeyPoint> & keypointsR, cv::Mat & descriptorsR, cv::Mat K, cv::Mat RTl, cv::Mat RTr, int imgRows)
{
    std::vector < std::vector < cv::Point_<float>>> matchedPoints(2);
    cv::Mat F, points3d;
    std::vector<cv::Mat> projMatr{ K * RTl, K * RTr };

    int maxYdif = 60;//calculate or take from the user in the future
    int ROIrows = 2 * maxYdif;
    //there exist 2 extra ROIs, so that exery point can be stored in two ROIs
    int numOfROIs = imgRows / maxYdif + 1;
    std::vector<std::vector<cv::KeyPoint>> ROIpointsL(numOfROIs);
    std::vector<cv::Mat> ROIdesL(numOfROIs);
    std::vector<std::vector<cv::KeyPoint>> ROIpointsR(numOfROIs);
    std::vector<cv::Mat> ROIdesR(numOfROIs);

    cerateROIs(keypointsL, descriptorsL, maxYdif, ROIpointsL, ROIdesL);
    cerateROIs(keypointsR, descriptorsR, maxYdif, ROIpointsR, ROIdesR);

    match(ROIpointsL, ROIdesL, ROIpointsR, ROIdesR, matchedPoints);
    crFundamental(RTl, RTr, K, F);
    cv::correctMatches(F, matchedPoints[0], matchedPoints[1], matchedPoints[0], matchedPoints[1]);
    determine3d(matchedPoints, projMatr, points3d);
}

int main()
{
    std::string directory = "C:/Users/Wojtas/Downloads/data-20211203T205153Z-001/data";

    cv::Mat K;
    std::vector<float> D;
    int imgRows;
    cameraIntristic(directory + "/camera_calibration.xml", K, D, imgRows);
    std::vector<std::string> fn;
    cv::glob(directory + "/*.JPG", fn, false);
    int numOfImgs = fn.size();

    std::vector<cv::Mat> RT(numOfImgs);
    cameraExtrinsics(directory + "/exterior_orientation.txt", RT);

    std::vector<std::vector<cv::KeyPoint>> keypoints;
    std::vector <cv::Mat> descriptors;

    std::vector<std::thread> Thr;
    for (int i = 0; i < numOfImgs; i++)
    {
        Thr.push_back(std::thread(findPoints, fn[i], std::ref(K), std::ref(D),
            std::ref(keypoints[i]), std::ref(descriptors[i])));
    }
    for (int i = 0; i < Thr.size(); i++)
    {
        Thr[i].join();
    }
    Thr.clear();

    std::vector < std::vector <std::vector < cv::Point_<float>>>> matchedPoints(numOfImgs - 1, std::vector <std::vector < cv::Point_<float>>>(2));
    for (int i = 0; i < numOfImgs - 1; i++)
    {
        Thr.push_back(std::thread(reconstructPoints, std::ref(keypoints[i]), std::ref(descriptors[i]), std::ref(keypoints[i + 1]), std::ref(descriptors[i + 1]), std::ref(K), std::ref(RT[i]), std::ref(RT[i + 1]), imgRows));
    }
    for (int i = 0; i < Thr.size(); i++)
    {
        Thr[i].join();
    }
    Thr.clear();

    ////////need to take all the points from reconstructPoints in the way that it doesn't produce multiple times the same points

    return 0;
}