//
// Created by sin on 2020/8/26.
//

#ifndef GROUND_FILTER_SHARED_RAY_GROUND_FILTER_H
#define GROUND_FILTER_SHARED_RAY_GROUND_FILTER_H

#include "ground_filter_core.h"
#include <vector>
#include <string>
using namespace std;

class App{
private:
    using Point = pcl::PointXYZI;
    RayGroundFilter filter;
    void filterFile(const string& filename, const string& outdir);
    void filterFolder(const string& foldername, const string& outdir);
    void extractPointsVec(const string& filename, vector<float> &vec);
    void saveBin(const vector<float> &out, const string &outFileName);
public:
    App(){};
    ~App(){};

    void Filter(const string& name, const string& outdir);
};

#endif //GROUND_FILTER_SHARED_RAY_GROUND_FILTER_H
