/*
 * @Description: 读写文件管理
 * @Author: Zhang Jun
 * @Date: 2020-07-24 19:22:53
 */
#ifndef LIO_SLAM_TOOLS_FILE_MANAGER_HPP_
#define LIO_SLAM_TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

namespace lio_slam {
class FileManager{
  public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool InitDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string directory_path);
};
}

#endif
