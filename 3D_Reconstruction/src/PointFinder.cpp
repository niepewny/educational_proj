#include "PointFinder.h"

cv::Ptr<cv::ORB> PointFinder::ORB;
cv::Mat PointFinder::K;
std::vector<double> PointFinder::D;
int PointFinder::gaussianSize = 1;
double PointFinder::gaussianSigma = 0.1;
bool PointFinder::init[2] = {1, 1};
bool PointFinder::applyNext;

PointFinder::PointFinder() {}

PointFinder::PointFinder(std::string path, cv::Mat& k, std::vector<double>& d)
{
    K = k;
    D = d;

    ORB = cv::ORB::create(1000000, 1.1f, 60, 5,
        0, 2, cv::ORB::HARRIS_SCORE, 5, 20);

    imgOrg = cv::imread(path);

    if (imgOrg.size == 0)
    {
        std::cout << "Image reading error :(" << std::endl;
        exit(0);
    }
}

void PointFinder::apply(std::vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptors)
{
    if (init[0])
    {
        preprocess();
        init[1] = 1;
    }
    if (init[1])
    {
        ORB->detect(img, keyPoints);

        for (int i = 0; i < keyPoints.size(); i++)
        {
            keyPoints[i].angle = 0;
        }

        ORB->compute(img, keyPoints, descriptors);

        init[0] = 0;
        init[1] = 0;
        applyNext = 1;
    }
}

void PointFinder::setGaussianSize(int size)
{
    if (gaussianSize != size)
    {
        gaussianSize = size;
        init[0] = 1;
    }
}
void PointFinder::setGaussianSigma(double sigma)
{
    if (gaussianSigma != sigma)
    {
        gaussianSigma = sigma;
        init[0] = 1;
    }
}
void PointFinder::setORBscaleFactor(float scaleFactor)
{
    if (ORB->getScaleFactor() != scaleFactor)
    {
        ORB->setScaleFactor(scaleFactor);
        init[1] = 1;
    }
}
void PointFinder::setORBnLevels(int nLevels)
{
    if (ORB->getNLevels() != nLevels)
    {
        ORB->setNLevels(nLevels);
        init[1] = 1;
    }
}
void PointFinder::setORBedgeThreshold(int threshold)
{
    if (ORB->getEdgeThreshold() != threshold)
    {
        ORB->setEdgeThreshold(threshold);
        init[1] = 1;
    }
}
void PointFinder::setORBpatchSize(int size)
{
    if (ORB->getPatchSize() != size)
    {
        ORB->setPatchSize(size);
        init[1] = 1;
    }
}
void PointFinder::setORBfastThreshold(int threshold)
{
    if (ORB->getFastThreshold() != threshold)
    {
        ORB->setFastThreshold(threshold);
        init[1] = 1;
    }
}
void PointFinder::setORBfirstLevel(int firstLevel)
{
    if (ORB->getFirstLevel() != firstLevel)
    {
        ORB->setFirstLevel(firstLevel);
        init[1] = 1;
    }
}

void PointFinder::preprocess()
{
    cv::undistort(imgOrg.clone(), img, K, D);

    cv::cuda::GpuMat GPUimg(img);
    cv::cuda::cvtColor(GPUimg, GPUimg, cv::COLOR_RGB2GRAY);

    cv::Ptr<cv::cuda::CLAHE> clahe = cv::cuda::createCLAHE();
    clahe->setClipLimit(100);
    clahe->apply(GPUimg, GPUimg);

    cv::Ptr<cv::cuda::Filter> gaussian = cv::cuda::createGaussianFilter(GPUimg.type(), GPUimg.type(), cv::Size(gaussianSize, gaussianSize), gaussianSigma);
    gaussian->apply(GPUimg, GPUimg);

    GPUimg.download(img);
}
