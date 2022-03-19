#include "CloudMaker.h"
#include <math.h>

    pcl::visualization::CloudViewer viewer("Cloud");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cl(new pcl::PointCloud<pcl::PointXYZ>);

    void deleteCl()
    {
        cl.~shared_ptr();
        //delete cl;
    }

    void CloudMaker::skipData(std::ifstream& file, int numOfData, bool line)
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
    template <typename T>
    void CloudMaker::mat2vec(cv::Mat& mat, std::vector<T>& vec)
    {
        for (int i = 0; i < mat.rows; i++)
        {
            vec.push_back(mat.at<T>(i, 0));
        }
    }

    void CloudMaker::cameraIntristic(std::string path, cv::Mat& K, std::vector<double>& D, int& imgCols, int& imgRows)
    {
        cv::FileStorage file;
        cv::Mat Dmat;

        file.open(path, cv::FileStorage::READ);
        file["Camera_Matrix"] >> K;
        file["Distortion_Coefficients"] >> Dmat;
        file["image_Height"] >> imgRows;
        file["image_Width"] >> imgCols;

        file["image_Width"] >> imgCols;
        file["image_Width"] >> imgCols;

        file.release();
        mat2vec<double>(Dmat, D);
    }

    void CloudMaker::cameraExtrinsics(std::string path, std::vector<cv::Mat>& RT)
    {
        std::vector<cv::Mat> C(RT.size());

        std::ifstream file;
        file.open(path);

        if (!file)
        {
            std::cout << "Bad extrinsics .txt file" << std::endl;
            exit(1);
        }

        skipData(file, 2, 1);

        for (int i = RT.size() - 1; i >= 0; i--)
        {
            C[i] = cv::Mat::zeros(3, 1, CV_64F);

            RT[i] = cv::Mat::zeros(3, 3, CV_64F);

            skipData(file, 1);

            for (int j = 0; j < 3; j++)
            {
                file >> C[i].at<double>(j, 0);
            }
            C[i].at<double>(1, 0) = -C[i].at<double>(1, 0);

            skipData(file, 3);

            for (int j = 0; j < 3; j++)
            {
                for (int k = 0; k < 3; k++)
                {
                    file >> RT[i].at<double>(j, k);
                }
            }
            C[i] = RT[i] * C[i];//-

            cv::hconcat(RT[i], C[i], RT[i]);
        }

        file.close();
    }

    void CloudMaker::makeFinder(PointFinder& pointFinder, std::string path, cv::Mat K, std::vector<double> D)
    {
        pointFinder = PointFinder::PointFinder(path, K, D);
    }

    void CloudMaker::exportPoints(std::vector<cv::Mat> points3d, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        cloud->clear();
        cv::Vec3f data;
        for (int i = 0; i < points3d.size(); i++)
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
    }

    CloudMaker::CloudMaker(std::string directory)
    {
        cv::Mat K;
        std::vector<double> D;
        int imgCols, imgRows;
        std::vector<std::string> fn;

        cv::glob(directory + "/*.JPG", fn, false);
        int numOfImgs = fn.size();

        keypoints = std::vector<std::vector<cv::KeyPoint>>(numOfImgs);
        descriptors = std::vector <cv::Mat>(numOfImgs);
        points3d = std::vector<cv::Mat>(numOfImgs - 1);
        cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        cameraIntristic(directory + "/camera_calibration.xml", K, D, imgCols, imgRows);

        std::vector<cv::Mat> RT(numOfImgs);
        cameraExtrinsics(directory + "/exterior_orientation.txt", RT);

        //creating PointFinders (chwilowo poza threadem)
        std::vector<std::thread> Thr;
        pointFinder = std::vector<PointFinder>(numOfImgs);

        for (int i = 0; i < numOfImgs; i++)
        {            
            pointFinder[i] = PointFinder::PointFinder(fn[i], K, D);
        }

        for (int i = 0; i < Thr.size(); i++)
        {
            Thr[i].join();
        }
        Thr.clear();

        //creating PointReconstructors
        for (int i = 0; i < numOfImgs - 1; i++)
        {
            pointReconstructor.push_back(PointReconstructor::PointReconstructor(K, RT[i], RT[i + 1], imgCols, imgRows));
        }

        //creating filter and viewer
        filter = pcl::RadiusOutlierRemoval<pcl::PointXYZ>(true);
    }

    CloudMaker::~CloudMaker()
    {
        deleteCl();
    }

    void CloudMaker::apply()
    {
        std::vector<std::thread> Thr;

        for (int i = 0; i < pointFinder.size(); i++)
        {
            Thr.push_back(std::thread(&PointFinder::apply, &pointFinder[i], std::ref(keypoints[i]), std::ref(descriptors[i])));
        }
        for (int i = 0; i < Thr.size(); i++)
        {
            Thr[i].join();
        }
        Thr.clear();

        for (int i = 0; i < pointReconstructor.size(); i++)
        {
            Thr.push_back(std::thread(&PointReconstructor::apply, &pointReconstructor[i], std::ref(keypoints[i]),
                std::ref(descriptors[i]), std::ref(keypoints[i + 1]), std::ref(descriptors[i + 1]), std::ref(points3d[i]), PointFinder::applyNext));
        }
        for (int i = 0; i < Thr.size(); i++)
        {
            Thr[i].join();
        }
        PointFinder::applyNext = 0;
        Thr.clear();

        if (PointReconstructor::applyNext)
        {
            exportPoints(points3d, cloud);
            PointReconstructor::applyNext = 0;
        }

        filter.setInputCloud(cloud);
        filter.filter(*cl);
        viewer.showCloud(cl);
    }