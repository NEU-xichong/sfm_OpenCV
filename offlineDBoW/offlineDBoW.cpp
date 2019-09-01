//
// Created by xc on 19-5-30.
//
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>

#include "DBoW3/DBoW3.h"

using namespace std;
using namespace cv;
using namespace boost::filesystem;
using namespace DBoW3;


int main(int argc ,char**argv)
{
    if(argc!=2)
    {
        cerr<<"Parameter input error!"<<endl;
        return -1;
    }

    path dirPath(argv[1]);
    if(not exists(dirPath) or not is_directory(dirPath))
    {
        cerr<<"File cannot be opened!"<<endl;
        return -1;
    }

    vector <Mat> descriptors;

    //extract sift feature
    cout<<"detecting SIFT features..."<<endl;
    int count=0;

    for(directory_entry& imgPath:directory_iterator(dirPath))
    {
        count++;
        cout<<"image: "<<count<<'\n'<<"imgDir: "<<imgPath<<endl;

        string extension=imgPath.path().extension().string();

        boost::algorithm::to_lower(extension);

        if(extension==".jpg" or extension==".jpeg" or extension==".png")
        {
            Mat img=imread(imgPath.path().string());
            Mat decs;
            Ptr<cv::xfeatures2d::SIFT> ptr=cv::xfeatures2d::SIFT::create();
            vector<KeyPoint> temp_kpts;
            ptr->detect(img,temp_kpts);

            cout<<temp_kpts.size()<<endl;

            ptr->compute(img,temp_kpts,decs);

            descriptors.push_back(decs);

        }
    }
    cout<<"sift done"<<endl;
    //create vocabulary

    cout<<"creating vocabulary..."<<endl;
    //const WeightingType weight = TF_IDF;
   // const ScoringType score = L1_NORM;
  /*  DBoW3::Vocabulary vocab(10,6);

    vocab.create(descriptors);

    cout<<"vocabulary info: "<<vocab<<endl;

    vocab.save("vocabulary.yml.gz");

    cout<<"vocabulary done!"<<endl;*/
    return 0;
}

