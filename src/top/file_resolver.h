#pragma once

#include <fstream>
#include <iostream>
#include <streambuf>
#include <lua.hpp>
#include <map>
#include <memory>

#include "glog/logging.h"

constexpr char kConfigurationFilesDirectory[] = "/usr/local/share/cartographer/configuration_files";
constexpr char kSourceDirectory[] = "";

class FileResolver
{
public:
  virtual ~FileResolver() {}
  FileResolver(
      const std::vector<std::string> &configuration_files_directories)
      : configuration_files_directories_(configuration_files_directories)
  {
    configuration_files_directories_.push_back(kConfigurationFilesDirectory);
  }

  std::string GetFullPathOrDie(const std::string &basename)
  {
    for (const auto &path : configuration_files_directories_)
    {
      const std::string filename = path + "/" + basename;
      std::ifstream stream(filename.c_str());
      if (stream.good())
      {
        LOG(INFO) << "Found '" << filename << "' for '" << basename << "'.";
        return filename;
      }
    }
    LOG(FATAL) << "File '" << basename << "' was not found.";
  }
  std::string GetFileContentOrDie(const std::string &basename)
  {
    const std::string filename = GetFullPathOrDie(basename);
    std::ifstream stream(filename.c_str());
    return std::string((std::istreambuf_iterator<char>(stream)),
                       std::istreambuf_iterator<char>());
  }

private:
  std::vector<std::string> configuration_files_directories_;
};

