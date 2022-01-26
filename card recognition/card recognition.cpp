#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

//funkcja przyjmuje od u¿ytkownika œcie¿kê do obrazka
string path()
{
    std::string filePath;
    std::cout << "podaj sciezke do obrazka" << std::endl;
    std::getline(std::cin, filePath);
    filePath.erase(std::remove(filePath.begin(), filePath.end(), 34), filePath.end());
    for (int i = 0; i < filePath.size(); i++)
        if (filePath[i] == 92)
            filePath[i] = '/';
    return filePath;
}

//function takes as an input choosen colour
int choose()
{
    char ch = '_';
    while (ch < '0' || ch > '4')
    {
        cout << "podaj kolor: \n 1: czerwony \n 2: zolty \n 3: zielony \n 4: niebieski \n";
        cin >> ch;
    }
    int choice = ch - '0';

    return choice;
}

//function detects horizontal or vertical lines (depends on kernelSize). Output is the number of nonzero pixels
int detectLines(Mat imgRaw, Size kernelSize)
{
    Mat img;
    imgRaw.copyTo(img);

    erode(img, img, getStructuringElement(MORPH_RECT, kernelSize));
    dilate(img, img, getStructuringElement(MORPH_RECT, kernelSize));

    return countNonZero(img);
}

//counts pixels of different hue values. Output is the id of most numerous value (scale 0-19)
int specifyColor(Mat img)
{    
    Mat imgHSV;
    cvtColor(img, imgHSV, COLOR_BGR2HSV);
    Mat splitted[3];
    split(imgHSV, splitted);

    int histSize[] = { 20 };
    float h_ranges[] = { 0, 180 };

    const float* ranges[] = { h_ranges };
    int channels[] = { 0 };

    MatND hist;
    int yellowPx = 1e4;
    calcHist(&splitted[0], 1, channels, Mat(), hist, 1, histSize, ranges);
    hist.at<float>(2, 0) -= yellowPx;
    Point maxLoc;
    minMaxLoc(hist, NULL, NULL, NULL, &maxLoc);
    return maxLoc.y;
}

//finds circles currently only corners of card
vector<Point> circlesDetect(Mat img)
{
    int circleThresh = 5;
    int minDist = 30;
    int minRadious = 35;
    int maxRadious = 50;

    vector<Vec3f> circles;
    HoughCircles(img, circles, HOUGH_GRADIENT, 1, minDist, 1, circleThresh, minRadious, maxRadious);

    vector<Point>centers;
    for (size_t i = 0; i < circles.size(); i++)        
    {
        centers.push_back(Point(circles[i][0], circles[i][1]));
    }

    return centers;
}

//calculates circularity
double calculateCircularity(vector<Point> contour, Mat img)
{
    double area = contourArea(contour);
    double contourLength =arcLength(contour, true);
    double circularity = 4 * CV_PI * area / (contourLength * contourLength);

    return circularity;
}

//leaves only contours on the image, returns vector of contour
void extractContours(Mat& img, vector<vector<Point>> &contour)
{
    cvtColor(img, img, cv::COLOR_BGR2GRAY);
    GaussianBlur(img, img, Size(11, 11), 9);
    adaptiveThreshold(img, img, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 21, 0);
    medianBlur(img, img, 11);
    erode(img, img, getStructuringElement(MORPH_RECT, Size(7, 7)));
    dilate(img, img, getStructuringElement(MORPH_RECT, Size(11, 11)));


    Mat clean = Mat::zeros(img.size(), CV_8U);
    vector<vector<Point>>contours;

    findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int max = 0;
    int maxId = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        auto area = contourArea(contours[i]);
        auto moment = moments(contours[i]);
        auto center = Point2d{ moment.m10 / moment.m00,moment.m01 / moment.m00 };
        double  dist = norm(center - Point2d(img.cols / 2, img.rows / 2));

        if (max < area && dist < 30)
        {
            max = area;
            maxId = i;
        }
            
    }
    contour.push_back(contours[maxId]);

    drawContours(clean, contour, -1, Scalar(255, 255, 0), 5);
    img = clean;
}

//"translates" specific values to card type. Commented lines show user see every output (if uncommented)
void specifyCard(int colorId, double circularity, int linesHor, int linesVert, int &type, int &colour)
{
    if (colorId < 2) cout << "red ";
    else if (colorId == 2) cout << "yellow ";
    else if (colorId == 3 || colorId == 4) cout << "green ";
    else cout << "blue ";

    if (linesHor > 220 && circularity < 0.6) cout << "5" << endl; //
    else if (linesVert > 500 && circularity < 0.6) cout << "1" << endl;
    else if (circularity > 0.7 && circularity < 0.8) cout << "8" << endl;
    else  cout << "spec" << endl;

    if (colorId < 2) colour = 1;
    else if (colorId == 2) colour = 2;
    else if (colorId == 3 || colorId == 4) colour = 3;
    else colour = 4;

    if (linesHor > 120 && circularity < 0.6) type = 5;
    else if (linesVert > 210 && circularity < 0.6) type = 1;
    else if (circularity > 0.7 && circularity < 0.8) type = 8;
    else  type = 0;

}

//finds corners of cards
void findCards(Mat img, vector<vector<Point2f>> &corners)
{
    cvtColor(img, img, cv::COLOR_BGR2GRAY);
    threshold(img, img, 25, 255, THRESH_BINARY);
    dilate(img, img, getStructuringElement(MORPH_ELLIPSE, Size(15, 15)));

    vector<Point>circles = circlesDetect(img);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < circles.size(); i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (pointPolygonTest(contours[j], circles[i], 0) >= 0)
            {
                corners[j].push_back(circles[i]);
                break;
            }
        }
    }
}

//extacts cards and rotates them vertically
void extractCard(Mat img, Mat &imgWarp, vector<Point2f> corners)
{
    int xShift = -15; 
    int yShift = -50; 
    sort(corners.begin(), corners.end(), [corners](Point2f pt1, Point2f pt2)
        {
            double dist1 = norm(corners[0] - pt1);
            double dist2 = norm(corners[0] - pt2);
            return(dist1 < dist2);
        });
    vector<Point2f>dst
    {
        Point(xShift, yShift),
        Point(norm(corners[0] - corners[1]) + xShift ,yShift),
        Point(xShift, norm(corners[0] - corners[2]) + yShift),
        Point(norm(corners[0] - corners[1]) + xShift, norm(corners[0] - corners[2]) + yShift)
    };
    Mat trans = getPerspectiveTransform(corners, dst);
    warpPerspective(img, imgWarp, trans, Size(norm(corners[0] - corners[1]) + 2 * xShift, norm(corners[0] - corners[2]) + 2 * yShift));
}

//in order to check every photo, change "all" value to 1 and path in "glob". Remember, that in folder should be only photos with 4 cards;
int main()
{
    int maxI;
    bool all = 1;
    vector<string> fn;

    if (all)
    {
        glob("./cards/*.png", fn, false);
        maxI = fn.size();
    }
    else
    {
        maxI = 1;
    }


    for(int i = 0; i < maxI; i++)
    {

        int sum = 0;
        Mat img;


        if (all)
        {
            img = imread(fn[i]);

            if (i % 4 == 0)
            {
                cout << "________NEW CARDS_______" << endl;
            }
            cout << "_______________" << endl;
            cout << "img " << i + 1 << endl;
        }

        if (!all)
        {
            do
            {
                img = imread(path());
            } while (img.cols == 0);
        }
        Mat imgRaw;
        img.copyTo(imgRaw);

        int choice = choose();

        resize(img, img, Size(), 0.5, 0.5);
        medianBlur(img, img, 15);

        vector<vector<Point2f>> corners(4);
        findCards(img, corners);

        for (int j = 0; j < 4; j++)
        {
            Mat imgWarp;
            extractCard(img, imgWarp, corners[j]);

            int colorId = specifyColor(imgWarp);
            vector<vector<Point>> contour;

            extractContours(imgWarp, contour);
            double circularity = calculateCircularity(contour[0], imgWarp);
            int linesVert = detectLines(imgWarp, Size(1, 60));
            int linesHor = detectLines(imgWarp, Size(50, 1));

            int type, colour;
            specifyCard(colorId, circularity, linesHor, linesVert, type, colour);
            if (type)
            {
                if (colour == choice)
                {
                    sum += type;
                }
            }
            else
            {
                double x = 0;
                double y = 0;
                for (int i = 0; i < corners[j].size(); i++)
                {
                    x += corners[j][i].x;
                    y += corners[j][i].y;
                }
                x = x / corners[j].size();
                y = y / corners[j].size();

                cout << "special card: \n ";
                cout << "x: " << x * 2 << endl;
                cout << "y: " << y * 2 << endl;
            }
        }
        cout << "sum == " << sum << endl;

    }
    return 0;
}