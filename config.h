//
// Created by sin on 2020/8/26.
//

#ifndef GROUND_FILTER_SHARED_CONFIG_H
#define GROUND_FILTER_SHARED_CONFIG_H

#include <string>
#include <map>
namespace rr
{
    class Config
    {
    public:
        Config()
        {
        }
        ~Config()
        {
        }
        bool ReadConfig(const std::string &filename);
        std::string ReadString(const char *section, const char *item, const char *default_value);
        int ReadInt(const char *section, const char *item, const int &default_value);
        float ReadFloat(const char *section, const char *item, const float &default_value);

    private:
        bool IsSpace(char c);
        bool IsCommentChar(char c);
        void Trim(std::string &str);
        bool AnalyseLine(const std::string &line, std::string &section, std::string &key, std::string &value);

    private:
        //std::map<std::string, std::string> settings_;
        std::map<std::string, std::map<std::string, std::string>> settings_;
    };
} // namespace rr

#endif //GROUND_FILTER_SHARED_CONFIG_H
