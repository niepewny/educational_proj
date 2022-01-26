#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/photo.hpp>
#include <algorithm>


void showpicture(cv::Mat img, std::string name)
{
    cv::namedWindow(name);
    cv::imshow(name, img);
    system("cls");
}

//funkcja przyjmuje od uzytkownika sciezke do pliku, a takze ja przetwarza
std::string path()
{
    std::string filePath;
    std::cout << "podaj sciezke do obrazka" << std::endl;
    std::cin >> filePath;
    filePath.erase(std::remove(filePath.begin(), filePath.end(), 34), filePath.end());
    for (int i = 0; i < filePath.size(); i++)
        if (filePath[i] == 92)
            filePath[i] = '/';
    return filePath;
}

//funkcja przyjmuje od uzytkownika info o typie projektu. 
int ch_project()
{
    int choice;
    std::cout << "wybierz projekt" << std::endl << "1: PR6" << std::endl << "2: MO1" << std::endl;
    std::cin >> choice;
    return choice;
}

//klasy zawieraja kody do obu projektow
class MO1
 {
 public:

    MO1(cv::Mat input, cv::Mat output)
    {
        int binarization, type, iterations;
        do 
        {
            std::cout << "czy obraz jest zbinaryzowany? Wybierz -1-, jesli obraz wymaga binaryzacji, lub -0- w przeciwnym wypadku " << std::endl;
            std::cin >> binarization;
        } while (binarization != 0 && binarization != 1);
        do
        {
            std::cout << "czy chcesz wykonac otwarcie -0-, czy zamkniecie -1-?" << std::endl;
            std::cin >> type;
        } while (type != 0 && type != 1);
        do
        {
            std::cout << "ile razy powtorzyc??" << std::endl;
            std::cin >> iterations;
        } while (iterations<=0);
        if (binarization)
        {
            adaptiveThreshold(input, input, 255, cv::THRESH_BINARY, cv::ADAPTIVE_THRESH_MEAN_C, 11, 0);
        }
        if (type == 0)
        {
            otwarcie(input, output, iterations);
        }
        else
        {
            zamkniecie(input, output, iterations);
        }

    }

 private:
    
     //funkcja przyjmuje od uzytkownika wszystkie dane kernela i zwraca go jako obraz
    cv::Mat Kernel()
    {
        int shape, size1, size2;
        do
        {
            std::cout << "podaj typ kernela (0/1/2 ~ rectangle/ceoss/ellipse) oraz jego wymiary oddzielone spacja."<<std::endl<< "Wymiary powinny byc nieparzyste, wieksze, niz 1." << std::endl;
            std::cin >> shape >> size1 >> size2;
        } while (shape != 0 && shape != 1 && shape != 2 || size1 % 2 == 0 || size2 % 2 == 0 || size1 == 1 || size2 == 1);

        cv::Mat kernel = getStructuringElement(shape, cv::Size(size1, size2));
        return kernel;
    }
    
    //funkcja dokonuje dylatacji lub erozji, jako, ¿e dylatacja oraz funkcja roznia sie tym, co uznajemy za tlo, a co za obiekt, wystarczy podac argument o wartosci obiektu
    // "0", by zmienic typ operacji. Domyslnie obiekt jest pikselem o maksymalnej intensywnosci
    void dilatation(cv::Mat input, cv::Mat output, cv::Mat kernel, unsigned char object = 255)
    {
        for (int i = 0; i < input.rows; i++)
        {
            for (int j = 0; j < input.cols; j++)
            {
                if (input.at<unsigned char>(i, j) == object)
                {
                    for (int k = 0; k < kernel.rows; k++)
                    {
                        for (int l = 0; l < kernel.cols; l++)
                        {
                            if (i - kernel.rows/2 + k >= 0 && j - kernel.cols / 2 + l >= 0 && i - kernel.rows/2 + k < input.rows && j - kernel.cols / 2 + l < input.cols && kernel.at<unsigned char>(k,l) !=0)
                            {
                                output.at<unsigned char>(i - kernel.rows / 2 + k, j - kernel.cols / 2 + l) = object;
                            }
                        }

                    }
                }
            }
        }
    }

    void otwarcie(cv::Mat input, cv::Mat output, int iterations)
    {
        cv::Mat kernel = Kernel();

        for (int i = 0; i < iterations; i++)
        {
            dilatation(input, output, kernel, 0);
            input = output.clone();
            dilatation(input, output, kernel);
            input = output.clone();
        }
    }

    void zamkniecie(cv::Mat input, cv::Mat output, int iterations)
    {
        cv::Mat kernel = Kernel();

        for (int i = 0; i < iterations; i++)
        {
            dilatation(input, output, kernel);
            input = output.clone();
            dilatation(input, output, kernel, 0);
            input = output.clone();
        }
    }
};

class PR6
{

public:

    PR6(cv::Mat input, cv::Mat output)
    {
        int block_size, strategy;
        do
        {
            std::cout << "wybierz nieparzysty rozmiar otoczenia" << std::endl;
            std::cin >> block_size;
        } while (block_size % 2 == 0);
        do
        {
            std::cout << "strategie wyznaczania progu:" << std::endl << "-1- srednia arytmetyczna" << std::endl << "-2- srednia geometryczna" << std::endl << "-3- srednia mediana" << std::endl;
            std::cin >> strategy;
        } while (strategy != 1 && strategy != 2 && strategy != 3);
        adaptive_threshold(input, output, block_size, strategy);
    }

private:

    unsigned char Threshold(std::vector<unsigned char> values, unsigned char strategy)
    {
        double threshold;

        switch (strategy)
        {
        case 1:
            threshold = 0;
            for (unsigned char i : values) threshold += i;
            threshold /= values.size();
            break;

        case 2:
            threshold = 1;
            for (double i : values)
                threshold *= pow((double)(i + 1), 1.0 / values.size());
            threshold -= 1;
            break;
        case 3:
            std::sort(values.begin(), values.end());
            if (values.size() % 2 == 1)
            {
                threshold = values.at((values.size() + 1) / 2);
            }
            else
            {
                threshold = ((float)(values.at(values.size() / 2) + values.at(values.size() / 2 + 1))) / 2;
            }
            break;
        }
        return threshold;
    }

    void adaptive_threshold(cv::Mat input, cv::Mat output, int block_size, unsigned char strategy)
    {
        int distance = block_size / 2;

        for (int i = 0; i < input.rows; i++)
        {
            for (int j = 0; j < input.cols; j++)
            {
                std::vector<unsigned char> values;
                for (int k = i - distance; k <= i + distance; k++)
                {
                    for (int l = j - distance; l <= j + distance; l++)
                    {
                        if (!(k < 0 || l < 0 || k > input.rows - 1 || l > input.cols - 1))  values.push_back(input.at<unsigned char>(k, l));
                    }
                }
                unsigned char threshold = Threshold(values, strategy);
                if (input.at<unsigned char>(i, j) >= threshold) output.at<unsigned char>(i, j) = 255;
                else output.at<unsigned char>(i, j) = 0;
                values.clear();
            }
        }
    }

};

int main()
{
    int project = ch_project();
    cv::Mat input = cv::imread(path(), cv::IMREAD_GRAYSCALE);
    cv::Mat output = input.clone();
    //podajê kopie, poniewa¿ algorytm przetwarza takze input. Algorytmy wyswietlaja outputy. 
    // Jesli uzytkownik chcialby otrzymac output, a nastepnie przeprowadzac na nim operacje, nalezy podac go jako drugi argument (referencja)    
    if (project == 1)
    {
        PR6 pr6(input.clone(), output);
    }
    else
    {
        MO1 mo1(input.clone(), output);
    }

    showpicture(output, "output");
    showpicture(input, "input");
    cv::waitKey(0);

    return 0;
}

