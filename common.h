//
// Created by sin on 2020/8/26.
//

#ifndef GROUND_FILTER_SHARED_COMMON_H
#define GROUND_FILTER_SHARED_COMMON_H

#include <string>
#include <vector>
using namespace std;

namespace Common{
    bool EndsWith(const string& str, const string& suffix);

    string Join(const string& folder, const string& file);

    bool Exist(const string& str);

    bool IsDir(const string& name);

    void MkDir(const string& name);
}

#endif //GROUND_FILTER_SHARED_COMMON_H
