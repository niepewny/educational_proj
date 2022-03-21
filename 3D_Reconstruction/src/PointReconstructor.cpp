#include "PointReconstructor.h"


    float PointReconstructor::Zmin;
    float PointReconstructor::Zmax;
    int PointReconstructor::ROIsize;
    int PointReconstructor::maxError;
    cv::Mat PointReconstructor::K;
    int PointReconstructor::imgRows;
    int PointReconstructor::imgCols;
    bool PointReconstructor::init[2] = { 1, 1 };
    bool PointReconstructor::applyNext = 1;


    PointReconstructor::PointReconstructor(cv::Mat k, cv::Mat rtl, cv::Mat rtr, int imgcols, int imgrows)
    {
        K = k;
        RTl = rtl;
        RTr = rtr;
        imgCols = imgcols;
        imgRows = imgrows;
        projMatr = { K * RTl, K * RTr };
        Pr2PlMat();
        crFundamental();

    }

    void PointReconstructor::apply(std::vector<cv::KeyPoint>& keypointsL, cv::Mat descriptorsL,
        std::vector<cv::KeyPoint>& keypointsR, cv::Mat& descriptorsR, cv::Mat& points3d, bool initAll)
    {
        if (init[0] || initAll)
        {
            ROIpointsR = std::vector<std::vector<std::vector<cv::KeyPoint>>>(imgCols / ROIsize, std::vector<std::vector<cv::KeyPoint>>(imgRows / ROIsize));
            ROIdesR = std::vector<std::vector<cv::Mat>>(imgCols / ROIsize, std::vector<cv::Mat>(imgRows / ROIsize));

            createROIs(keypointsR, descriptorsR);

            init[1] = 1;
        }
        if (init[1])
        {

            std::vector < std::vector < cv::Point_<double>>> matchedPoints(2);

            match(keypointsL, descriptorsL, matchedPoints);

            cv::correctMatches(F, matchedPoints[0], matchedPoints[1], matchedPoints[0], matchedPoints[1]);

            determine3d(matchedPoints, projMatr, points3d);

            init[0] = 0;
            init[1] = 0;
            applyNext = 1;
        }

    }

    void PointReconstructor::setZmin(float zmin)
    {
        if (Zmin != zmin)
        {
            Zmin = zmin;
            init[0] = 1;
        }
    }

    void PointReconstructor::setZmax(float zmax)
    {
        if (Zmax != zmax)
        {
            Zmax = zmax;
            init[0] = 1;
        }
    }

    void PointReconstructor::setROIsize(int roisize)
    {
        if (ROIsize != roisize)
        {
            ROIsize = roisize;
            init[0] = 1;
        }
    }

    void PointReconstructor::setMatcherMaxError(int maxerror)
    {
        if (maxError != maxerror)
        {
            maxError = maxerror;
            init[1] = 1;
        }
    }

    void PointReconstructor::mat2vec(cv::Mat& mat, std::vector<cv::Mat>& vec)
    {
        for (int i = 0; i < mat.cols; i++)
        {
            vec.push_back(mat.col(i).clone());
        }
    }

    void PointReconstructor::crFundamental()
    {
        cv::Mat R, T, Tx, Kinv;

        Kinv = K.inv();

        R = RTr2l(cv::Rect(0, 0, 3, 3));
        T = RTr2l.col(3);
        Tx = skew<double>(T);
        F = Kinv.t() * Tx * R * Kinv;
    }

    void PointReconstructor::Pr2PlMat()
    {
        cv::Mat hRT, hRTl, hRTr;
        cv::Mat hRow = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

        cv::vconcat(RTl, hRow, hRTl);
        cv::vconcat(RTr, hRow, hRTr);

        hRT = hRTl * hRTr.inv();
        RTr2l = hRT(cv::Rect(0, 0, 4, 3));

    }

    void PointReconstructor::createROIs(std::vector<cv::KeyPoint>& keypointsR, cv::Mat& descriptorsR)
    {        
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
        double B = RTr2l.at<double>(0, 3) * fx;
        double D = RTr2l.at<double>(2, 3);
        double F = RTr2l.at<double>(1, 3) * fy;

        for (int i = 0; i < keypointsR.size(); i++)
        {
            x = keypointsR[i].pt.x;
            y = keypointsR[i].pt.y;

            double A = RTr2l.at<double>(0, 0) * (x - cx) + RTr2l.at<double>(0, 1) * (y - cy) + RTr2l.at<double>(0, 2) * fx;
            double C = RTr2l.at<double>(2, 0) / fx * (x - cx) + RTr2l.at<double>(2, 1) / fy * (y - cy) + RTr2l.at<double>(2, 2);
            double E = RTr2l.at<double>(1, 0) * (x - cx) + RTr2l.at<double>(1, 1) * (y - cy) + RTr2l.at<double>(1, 2) * fy;

            for (float z = Zmin; z < Zmax; z += dZ)
            {

                u = ((A * z + B) / (C * z + D) + cx) / ROIsize;
                v = ((E * z + F) / (C * z + D) + cy) / ROIsize;

                if (u > 0 && u < (int)(imgCols / ROIsize) && v > 0 && v < (int)(imgRows / ROIsize) && !(u == old_u && v == old_v))
                {
                    ROIpointsR[u][v].push_back(keypointsR[i]);
                    ROIdesR[u][v].push_back(descriptorsR.row(i));
                    old_u = u;
                    old_v = v;
                }

                dZ = -(C * C * z * z * ROIsize + 2 * C * D * z * ROIsize + D * D * ROIsize)
                    / (-A * D + B * C + C * C * z * ROIsize + C * D * ROIsize);
                if (dZ < 0)
                {
                    dZ = -dZ;
                }
            }
        }
    }

    void PointReconstructor::match(std::vector<cv::KeyPoint>& keypointsL, cv::Mat& descriptorsL,
        std::vector < std::vector < cv::Point_<double>>>& matchedPoints)
    {
        cv::Ptr< cv::BFMatcher> matcherPtr = cv::BFMatcher::create(cv::NORM_HAMMING);
        std::vector<cv::DMatch> match;
        int u, v;
        for (int kpId = 0; kpId < keypointsL.size(); kpId++)
        {
            u = keypointsL[kpId].pt.x / ROIsize;
            v = keypointsL[kpId].pt.y / ROIsize;
            if (ROIpointsR[u][v].size())
            {
                cv::Mat A = descriptorsL.row(kpId);
                cv::Mat B = ROIdesR[u][v];
                matcherPtr->match(A, B, match);

                if ((match[0].distance < 2))
                {
                    matchedPoints[0].push_back(keypointsL[kpId].pt);
                    matchedPoints[1].push_back(ROIpointsR[u][v][match[0].trainIdx].pt);
                }
            }
        }
    }

    void PointReconstructor::determine3d(std::vector <std::vector < cv::Point_<double>>> matchedPoints, std::vector<cv::Mat>projMatr, cv::Mat& points3d)
    {
        cv::Mat hPoints(4, matchedPoints[0].size(), CV_64FC2);

        cv::triangulatePoints(projMatr[0], projMatr[1], matchedPoints[0], matchedPoints[1], hPoints);
        std::vector<cv::Mat>converter;
        hPoints = hPoints.t();
        mat2vec(hPoints, converter);
        cv::merge(converter, hPoints);

        cv::convertPointsFromHomogeneous(hPoints, points3d);
    }

    template <typename T>
    cv::Mat PointReconstructor::skew(cv::Mat V)
    {
        cv::Mat Vx = (cv::Mat_<T>(3, 3) <<
            0, -V.at<T>(2), V.at<T>(1),
            V.at<T>(2), 0, -V.at<T>(0),
            -V.at<T>(1), V.at<T>(0), 0);
        return Vx;
    }
