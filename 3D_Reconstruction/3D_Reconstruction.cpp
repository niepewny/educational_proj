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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>


void Pr2PlMat(cv::Mat RTl, cv::Mat RTr, cv::Mat &RTr2l)
{
    cv::Mat hRTr2l, hRTl, hRTr;
    cv::Mat hRow = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

    cv::vconcat(RTl, hRow, hRTl);
    cv::vconcat(RTr, hRow, hRTr);

    hRTr2l = hRTl * hRTr.inv();
    RTr2l = hRTr2l(cv::Rect(0, 0, 4, 3));
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

void mat2vec(cv::Mat& mat, std::vector<double>& vec)
{
    for (int i = 0; i < mat.rows; i++)
    {
        vec.push_back(mat.at<double>(i, 0));
    }
}

void cameraIntristic(std::string path, cv::Mat &K, std::vector<double>& D, int &imgCols, int &imgRows)
{
    cv::FileStorage file;
    cv::Mat Dmat;

    file.open(path, cv::FileStorage::READ);
    file["Camera_Matrix"] >> K;
    K.convertTo(K, CV_64F);
    file["Distortion_Coefficients"] >> Dmat;
    file["image_Height"] >> imgRows;
    file["image_Width"] >> imgCols;

    file["image_Width"] >> imgCols;
    file["image_Width"] >> imgCols;

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
        RT[i] = cv::Mat::zeros(3, 4, CV_64F);

        skipData(file, 1);

        for (int j = 0; j < 3; j++)
        {
            file >> RT[i].at<double>(j, 3);
            RT[i].at<double>(j, 3) = -RT[i].at<double>(j, 3);;
        }
        skipData(file, 3);

        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                file >> RT[i].at<double>(j, k);
            }
        }
    }

    file.close();
}

void preprocess(cv::Mat& img, cv::Mat &K, std::vector<double> &D) 
{
    cv::undistort(img.clone(), img, K, D);

    cv::cuda::GpuMat GPUimg(img);
    cv::cuda::cvtColor(GPUimg, GPUimg, cv::COLOR_RGB2GRAY);

    cv::Ptr<cv::cuda::CLAHE> clahe = cv::cuda::createCLAHE();
    clahe->setClipLimit(100);
    clahe->apply(GPUimg, GPUimg);

    cv::Ptr<cv::cuda::Filter> gaussian = cv::cuda::createGaussianFilter(GPUimg.type(), GPUimg.type(), cv::Size(9, 9), 5);
    gaussian->apply(GPUimg, GPUimg);

    GPUimg.download(img);
}

void cerateROIs(std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, cv::Mat RTl, cv::Mat RTr,  cv::Mat K, int ROIscale,
    std::vector<std::vector<std::vector<cv::KeyPoint>>> &ROIpoints, std::vector<std::vector<cv::Mat>> &ROIdes)
{
    cv::Mat RT;
    Pr2PlMat(RTl, RTr, RT);
    
    float Zmin = 1;
    float Zmax = 20;
    float dZ = 0;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    int u, v, x, y;
    int old_u = -1;
    int old_v = -1;
//We create ROIs based on possible areas, where one point can be on another img, as a function of Z coordinate. 
//Function has form of (Az+B)/(Cz+D)+cx
    double B = RT.at<double>(0, 3) * fx;
    double D = RT.at<double>(2, 3);
    double F = RT.at<double>(1, 3) * fy;

    for (int i = 0; i < keypoints.size(); i++)
    {
        x = keypoints[i].pt.x;
        y = keypoints[i].pt.y;

        double A = RT.at<double>(0, 0) * (x - cx) + RT.at<double>(0, 1) * (y - cy) + RT.at<double>(0, 2) * fx;
        double C = RT.at<double>(2, 0) / fx * (x - cx) + RT.at<double>(2, 1) / fy * (y - cy) + RT.at<double>(2, 2);
        double E = RT.at<double>(1, 0) * (x - cx) + RT.at<double>(1, 1) * (y - cy) + RT.at<double>(1, 2) * fy;

        for (float z = Zmin; z < Zmax; z += dZ)
        {

            u = ((A * z + B) / (C * z + D) + cx) / ROIscale;
            v = ((E * z + F) / (C * z + D) + cy) / ROIscale;

            if (u > 0 && u < 4000 / ROIscale && v > 0 && v < 3000 / ROIscale && !(u == old_u && v == old_v))
            {
                ROIpoints[u][v].push_back(keypoints[i]);
                ROIdes[u][v].push_back(descriptors.row(i));
                old_u = u;
                old_v = v;
            }
            dZ = -(C * C * z * z * ROIscale + 2 * C * D * z * ROIscale + D * D * ROIscale) 
                / (-A * D + B * C + C * C * z * ROIscale + C * D * ROIscale);
            if(dZ < 0)
            {
                dZ = -dZ;
            }
        }
    }
}

void match(std::vector<cv::KeyPoint> keypoints, cv::Mat descriptors, std::vector < std::vector<std::vector<cv::KeyPoint>>> ROIpoints,
    std::vector < std::vector<cv::Mat>> ROIdes, int ROIscale, std::vector < std::vector < cv::Point_<double>>> &matchedPoints)
{
    cv::Ptr< cv::BFMatcher> matcherPtr = cv::BFMatcher::create(cv::NORM_HAMMING);
    std::vector<cv::DMatch> match;
    int u, v;
    for (int kpId = 0; kpId < keypoints.size(); kpId++)
    {
        u = keypoints[kpId].pt.x / ROIscale;
        v = keypoints[kpId].pt.y / ROIscale;
        if (ROIpoints[u][v].size())
        {
            cv::Mat A = descriptors.row(kpId);
            cv::Mat B = ROIdes[u][v];
            matcherPtr->match(A, B, match);

            if ((match[0].distance < 2))
            {
                matchedPoints[0].push_back(keypoints[kpId].pt);
                matchedPoints[1].push_back(ROIpoints[u][v][match[0].trainIdx].pt);
            }
        }
    }
}

void determine3d(std::vector <std::vector < cv::Point_<double>>> matchedPoints, std::vector<cv::Mat>projMatr, cv::Mat &points3d)
{
    cv::Mat hPoints(4, matchedPoints[0].size(), CV_64FC2);

    cv::triangulatePoints(projMatr[0], projMatr[1], matchedPoints[0], matchedPoints[1], hPoints);
    std::vector<cv::Mat>converter;
    hPoints = hPoints.t();
    mat2vec(hPoints, converter);
    cv::merge(converter, hPoints);

    cv::convertPointsFromHomogeneous(hPoints, points3d);
}

void findPoints
(
    std::string path, cv::Mat K, std::vector<double>& D, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors
)
{
    cv::Ptr<cv::ORB> ORB = cv::ORB::create(1000000, 1.1f, 60, 5,
        0, 2, cv::ORB::HARRIS_SCORE, 5, 20);
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

void reconstructPoints(std::vector<cv::KeyPoint>& keypointsL, cv::Mat descriptorsL, std::vector<cv::KeyPoint>& keypointsR, cv::Mat& descriptorsR,
    cv::Mat K, cv::Mat RTl, cv::Mat RTr, int imgCols, int imgRows, cv::Mat &points3d)
{
    int ROIscale = 50;
    std::vector < std::vector < cv::Point_<double>>> matchedPoints(2);
    cv::Mat F;
    std::vector<cv::Mat> projMatr{ K * RTl, K * RTr };

    std::vector < std::vector<std::vector<cv::KeyPoint>>> ROIpoints(imgCols / ROIscale, std::vector<std::vector<cv::KeyPoint>>(imgRows / ROIscale));
    std::vector < std::vector<cv::Mat>> ROIdes(imgCols / ROIscale, std::vector<cv::Mat>(imgRows / ROIscale));

    cerateROIs(keypointsR, descriptorsR, RTl, RTr, K, ROIscale, ROIpoints, ROIdes);

    match(keypointsL, descriptorsL, ROIpoints, ROIdes, ROIscale, matchedPoints);

    determine3d(matchedPoints, projMatr, points3d);
}


int main()
{
    std::string directory;
    std::cout << "Where are the files?" << std::endl;
    std::cin >> directory;

    cv::Mat K;
    std::vector<double> D;
    int imgCols, imgRows;
    cameraIntristic(directory + "/camera_calibration.xml", K, D, imgCols, imgRows);
    std::vector<std::string> fn;
    cv::glob(directory + "/*.JPG", fn, false);
    int numOfImgs = fn.size();

    std::vector<cv::Mat> RT(numOfImgs);
    cameraExtrinsics(directory + "/exterior_orientation.txt", RT);

    std::vector<std::vector<cv::KeyPoint>> keypoints(numOfImgs);
    std::vector <cv::Mat> descriptors(numOfImgs);

    std::vector<std::thread> Thr;
    for (int i = 0; i < numOfImgs; i++)
    {
        Thr.push_back(std::thread(findPoints, fn[i], std::ref(K), std::ref(D), std::ref(keypoints[i]), std::ref(descriptors[i])));
    }
    for (int i = 0; i < Thr.size(); i++)
    {
        Thr[i].join();
    }
    Thr.clear();

    std::vector<cv::Mat> points3d(numOfImgs - 1);

    std::vector < std::vector <std::vector < cv::Point_<float>>>> matchedPoints(numOfImgs - 1, std::vector <std::vector < cv::Point_<float>>>(2));
    for (int i = 0; i < numOfImgs - 1; i++)
    {
        Thr.push_back(std::thread(reconstructPoints, std::ref(keypoints[i]), std::ref(descriptors[i]), std::ref(keypoints[i + 1]), std::ref(descriptors[i + 1]), std::ref(K), std::ref(RT[i]), std::ref(RT[i + 1]), imgCols, imgRows, std::ref(points3d[i])));
    }
    for (int i = 0; i < Thr.size(); i++)
    {
        Thr[i].join();
    }
    Thr.clear();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cv::Vec3f data;
    for(int i = 2; i<points3d.size(); i++)
        for (int j = 0; j < points3d[i].rows; j++)
        {
            points3d[i].convertTo(points3d[i], CV_32F);
            pcl::PointXYZ point;
            data = points3d[i].at<cv::Vec3f>(j, 0);
            point.x = data[0];
            point.y = data[1];
            point.z = data[2];
            cloud->points.push_back(point);
        }

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter(true);
    rorfilter.setInputCloud(cloud);
    rorfilter.setRadiusSearch(1);
    rorfilter.setMinNeighborsInRadius(30);
    rorfilter.filter(*cloud);

    pcl::visualization::CloudViewer viewer("Viewer");
    viewer.showCloud(cloud);

    while (1)
    {}

    return 0;
}