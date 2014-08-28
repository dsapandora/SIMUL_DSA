#include "cvaux.h"
#include "highgui.h"
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <ctype.h>
#include <iostream>
#include "ml.h"
#include <QtCore/QFileInfo>
#include <QtCore/QDir>
#include <QtCore/QStringList>
#include <QtCore/QtDebug>
using namespace cv;
using namespace std;
extern int dumpFeatures();


void dumpRHOG(int positive,QString traintest,QString BasePath)
{
    QString trainPath(BasePath+"/"+traintest);
    std::vector<string> real_labels;
    Size winStride(8,8), padding(0,0);
    HOGDescriptor hog;



    vector<Point> locations;
    vector<float> descriptors;


    QString typeTraining;

    if(positive == 1)
    {
       typeTraining = "pos";

    }else if(positive == -1){
        typeTraining = "neg";

    }else if(positive == 2){
        typeTraining = "hardneg";

    }

    QFileInfo labelsPath(trainPath+"/"+typeTraining+".lst");
    QDir train_path(trainPath+"/"+typeTraining);
    if (!labelsPath.isReadable())
    {
        cout << QString("file %1 is not readable").arg(labelsPath.filePath()).toStdString() << endl;
        exit(EXIT_FAILURE);
    }
    ifstream file;
    string lineread;

    file.open(labelsPath.filePath().toStdString().c_str(), ifstream::in);
    if(file.bad())
        qDebug() << "bad file";
    while(std::getline(file, lineread))
    {
        real_labels.push_back(lineread);
    }

    QStringList trainSets;
    QStringList trainList = train_path.entryList();
    for (int i = 0; i < trainList.size(); ++i)
    {
         QString trainSet = trainList.at(i);
         if (trainSet.startsWith("."))
             continue;
         trainSets.append(trainSet);

    }
    assert(trainSets.size()>0);
    QFile* features = new QFile("Features"+traintest+typeTraining+".txt");

        if (!features->open(QIODevice::WriteOnly | QIODevice::Text))
        {
            cout << "cant open feature file" << endl;
            exit(EXIT_FAILURE);
        }

    QTextStream outfeatures(features);
 //   for(unsigned int trainNum=0; trainNum < trainSets.size(); trainNum++) {


        //loop over images in folders
    // int trainNum = 1;
    //   QString trainSet = trainSets.at(trainNum);
       for(unsigned int i=0; i < real_labels.size(); i++)
       {






            QFileInfo picPath(train_path.path()+ QString("/%1").arg(trainSets.at(i)));
            assert(picPath.exists());

            string picPathString = picPath.filePath().toStdString();
            Mat handimg = cv::imread(picPathString, 0);
            cout <<picPathString<<endl;

            if (!handimg.data)
            {
                std::cout << "ERROR: can't read image: " << picPathString << std::endl;
                exit(EXIT_FAILURE);
            }

            hog.gammaCorrection = false;
            hog.derivAperture = false;

            std::cout << "blockSize           =("<< hog.blockSize.width <<","
             <<  hog.blockSize.height  <<")"<< endl;
            std::cout << "blockStride         =("  <<  hog.blockStride.width <<","
             <<  hog.blockStride.height<<")"<< endl;
            std::cout << "CellSize            =("<< hog.cellSize.width <<","
             <<    hog.cellSize.height <<")"<< endl;
            std::cout << "derivAperture bool  =" << hog.derivAperture << endl;
            std::cout << "gammaCorrection bool=" << hog.gammaCorrection << endl;
            std::cout << "histogramNormType   =" << hog.histogramNormType<< "if = 0 means L2Hys" << endl;
            std::cout << "L2HysTheshold       =" << hog.L2HysThreshold<< endl;
            std::cout << "nbins               =" << hog.nbins<< endl;
            //std::cout << "svmDetector         =" << hog.svmDetector<< endl;
            std::cout << "winSigma            =" << hog.winSigma<< endl;
            std::cout << "winSize             =(" <<   hog.winSize.width << ","
             << hog.winSize.height <<")" << endl;
            hog.compute(handimg, descriptors, winStride, padding, locations);
            if(positive==1)
                outfeatures<<1<<" ";
            else if(positive == -1 || positive ==2)
                outfeatures<<-1<<" ";
            for(vector<float>::iterator it=descriptors.begin();it!=descriptors.end();++it)
            {
                outfeatures << it-descriptors.begin()+1 <<":"<<*it <<" ";
            }
            outfeatures << endl;
            // initialize the matrix with info from first hand

          //  Mat descr(descriptors);
           // qDebug() << descr.size().width<<"x" << descr.size().height;

            //Mat t = train_mat.col(imgnum);
            //descr.copyTo(t);
        }
   // }
}

int main(int argc, char** argv)
{
    Mat img;
    int negpos;

    if( argc != 4 )
    {
        printf("Usage: peopledetect -n|-p train|test (BASE_TEST), %d\n",argc);
        return 1;
    }
    if(!strcmp(argv[1],"-n"))
        negpos=-1;
    else if(!strcmp(argv[1],"-p"))
        negpos=1;
    else if(!strcmp(argv[1],"-h"))
        negpos=2;
    else
    {
        printf("Usage: peopledetect -n|-p train|test (BASE_TEST)\n");
        return 2;
    }

    if(!(!strcmp(argv[2],"train") || !strcmp(argv[2],"test")))
    {
        printf("Usage: peopledetect -n|-p train|test (BASE_TEST)\n %s\n",argv[2]);
        return 3;
    }


   QString fName; fName.append(argv[3]);

    HOGDescriptor hog;

    qDebug() << fName << endl;
    dumpRHOG(negpos,argv[2],fName);


 //    dumpRHOG(2,"test",fName);


    namedWindow("people detector", 1);

    return 0;
}
