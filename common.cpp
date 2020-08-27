//
// Created by sin on 2020/8/26.
//
#include "common.h"
#include <sys/stat.h>
#include <cstdio>
#include <unistd.h>
#include <iostream>

namespace Common{
    bool EndsWith(const string& str, const string& suffix){
        if (str.empty()) return false;
        if (str.size() < suffix.size()) return false;
        return str.substr(str.size() - suffix.size(), suffix.size()) == suffix;
    }

    string Join(const string& folder, const string& file){
        if (EndsWith(folder, "/")){
            return folder + file;
        }else{
            return folder + "/" + file;
        }
    }

    bool Exist(const string& name){
        return access(name.c_str(), 0) == 0;
    }

    bool IsDir(const string& name){
        if(!Exist(name)){
            std::cout << "path: " << name << " doesn't exist" << endl;
            return false;
        }
        const char* tName = name.c_str();
        struct stat buf{};  // 用于保存文件信息
        stat(tName, &buf);  // 获取文件信息,保存到buf中
        return S_ISDIR(buf.st_mode);
    }

    void MkDir(const string& name){
        mkdir(name.c_str(), S_IRWXU);
    }
}


