//
// Created by sin on 2020/8/26.
//
#include "ground_filter_core.h"
#include "ray_ground_filter.h"
#include <pcl/conversions.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "common.h"
#include <dirent.h>
using namespace std;

int main(int argc, char* argv[]){
    if (argc < 3){
        cout << "usage:" << endl;
        cout << "    ./ground_filter <input file(dir)> <output dir>" << endl;
        exit(1);
    }
    string name = argv[1];
    string outdir = argv[2];

    App app;
    app.Filter(name, outdir);
    return 0;
}

void App::Filter(const string& name, const string& outdir) {
    assert(Common::Exist(name));
    if(!Common::Exist(outdir)){
        Common::MkDir(outdir);
    }
    if(Common::IsDir(name)){
        filterFolder(name, outdir);
    }else{
        filterFile(name, outdir);
    }
}

void App::extractPointsVec(const string& filename, vector<float> &vec){
//    string file = "/home/sin/Downloads/nuscene_test/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin";
    ifstream inFile(filename, ios::in|ios::binary);
    if (!inFile){
        cout << "inFile error" << endl;
        return;
    }
    vec.reserve(3* sizeof(inFile)/ sizeof(float)/5);
    float fea[5];
    int n = 0;
    try {
        while(inFile.read((char*)&fea[0], 5*sizeof(float))){
            vec.push_back(fea[0]);  // x
            vec.push_back(fea[1]);  // y
            vec.push_back(fea[2]);  // z
            ++n;
        }
    }catch (exception &e){
        cout << e.what() << endl;
    }
    inFile.close();
}

void App::saveBin(const vector<float> &out, const string &outFileName){
    ofstream outFile(outFileName, ios::out|ios::binary);
    if (!outFile){
        cout << "outFile error" << endl;
        return;
    }
    float fea[out.size()];
    for(int i = 0; i < out.size(); i++){
        fea[i] = out[i];
    }
    try {
        outFile.write((char*)&fea[0], out.size()* sizeof(float));
    }catch (exception &e){
        cout << e.what() << endl;
    }
    outFile.close();
}

void App::filterFile(const string& filename, const string& outdir) {
    cout << "[+]filterFile: " << filename << endl;
    // 解析名称
    string origin_whole_name = filename.substr(filename.rfind('/'));
    string origin_file_name = origin_whole_name.substr(0, origin_whole_name.rfind('.'));
    string out_filterd_pcd_name = Common::Join(outdir, origin_file_name) + "_filtered.pcd";
    string out_filtered_bin_name = Common::Join(outdir, origin_file_name) + "_filtered.bin";
    string out_origin_pcd_name = Common::Join(outdir, origin_file_name) + "_origin.pcd";
    cout << "    origin_whole_name: " << origin_whole_name << endl;
    cout << "     origin_file_name: " << origin_file_name << endl;
    cout << " out_filterd_pcd_name: " << out_filterd_pcd_name << endl;
    cout << "out_filtered_bin_name: " << out_filtered_bin_name << endl;
    cout << "  out_origin_pcd_name: " << out_origin_pcd_name << endl;

    std::vector<float> input, output;
    // 读取点云xyz到input中
    extractPointsVec(filename, input);
    cout << "Origin : total " << input.size() << " points" << endl;
    filter.DoFilter(input, output, out_origin_pcd_name, out_filterd_pcd_name);
    cout << "Filterd: total " << output.size() << " points" << endl;

    // 保存滤除地面后的点云到二进制bin文件
    saveBin(output, out_filtered_bin_name);
    cout << "[-]filterFile: " << filename << endl << endl;
}

void App::filterFolder(const string &foldername, const string &outdir) {
    // 读取文件夹下所有bin结尾的二进制文件
    DIR *dir;
    struct dirent *ptr;
    if ((dir = opendir(foldername.c_str())) == NULL)
    {
        perror("Open dir error...");
        std::cout << "Check: " << foldername << std::endl;
        exit(1);
    }
    vector<string> files;
    while ((ptr = readdir(dir)) != NULL)
    {
        if (ptr->d_type == 8 && Common::EndsWith(ptr->d_name, "bin"))
            files.emplace_back(ptr->d_name);
    }
    closedir(dir);
    cout << "Find " << files.size() << " bin files to handler" << endl;
    for(const string& filename: files){
        filterFile(Common::Join(foldername, filename), outdir);
    }
}

