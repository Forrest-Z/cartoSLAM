#pragma once

#include <fstream>
#include <iostream>
#include <streambuf>
#include <lua.hpp>
#include <map>
#include <memory>

#include "file_resolver.h"

#include "glog/logging.h"

class Lua_config_reader
{
public:
  Lua_config_reader(std::string file_name, std::unique_ptr<FileResolver> file_resolver);

private:
  std::string file_name_;

  lua_State *L_; // The name is by convention in the Lua World.
  int index_into_reference_table_;

  // This is shared with all the sub dictionaries.
  const std::shared_ptr<FileResolver> file_resolver_;

  // If true will check that all keys were used on destruction.

  // This is modified with every call to Get* in order to verify that all
  // parameters are read exactly once.
  std::map<std::string, int> reference_counts_;

  // List of all included files in order of inclusion. Used to prevent double
  // inclusion.
  std::vector<std::string> included_files_;
  static int LuaInclude(lua_State *L);
};
