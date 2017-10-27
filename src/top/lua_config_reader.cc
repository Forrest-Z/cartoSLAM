#include "lua_config_reader.h"

#include <algorithm>

void QuoteStringOnStack(lua_State *L)
{
    CHECK(lua_isstring(L, -1)) << "Top of stack is not a string value.";
    int current_index = lua_gettop(L);

    // S: ... string
    lua_pushglobaltable(L);        // S: ... string globals
    lua_getfield(L, -1, "string"); // S: ... string globals <string module>
    lua_getfield(L, -1,
                 "format");          // S: ... string globals <string module> format
    lua_pushstring(L, "%q");         // S: ... string globals <string module> format "%q"
    lua_pushvalue(L, current_index); // S: ... string globals <string module>
                                     // format "%q" string

    lua_call(L, 2, 1);             // S: ... string globals <string module> quoted
    lua_replace(L, current_index); // S: ... quoted globals <string module>

    lua_pop(L, 2); // S: ... quoted
}

// Sets the given 'dictionary' as the value of the "this" key in Lua's registry
// table.
void SetDictionaryInRegistry(lua_State *L, Lua_config_reader *dictionary)
{
    lua_pushstring(L, "this");
    lua_pushlightuserdata(L, dictionary);
    lua_settable(L, LUA_REGISTRYINDEX);
}

// Gets the 'dictionary' from the "this" key in Lua's registry table.
Lua_config_reader *GetDictionaryFromRegistry(lua_State *L)
{
    lua_pushstring(L, "this");
    lua_gettable(L, LUA_REGISTRYINDEX);
    void *return_value = lua_isnil(L, -1) ? nullptr : lua_touserdata(L, -1);
    lua_pop(L, 1);
    CHECK(return_value != nullptr);
    return reinterpret_cast<Lua_config_reader *>(return_value);
}

// CHECK()s if a Lua method returned an error.
void CheckForLuaErrors(lua_State *L, int status)
{
    CHECK_EQ(status, 0) << lua_tostring(L, -1);
}

// Returns 'a' if 'condition' is true, else 'b'.
int LuaChoose(lua_State *L)
{
    CHECK_EQ(lua_gettop(L), 3) << "choose() takes (condition, a, b).";
    CHECK(lua_isboolean(L, 1)) << "condition is not a boolean value.";

    const bool condition = lua_toboolean(L, 1);
    if (condition)
    {
        lua_pushvalue(L, 2);
    }
    else
    {
        lua_pushvalue(L, 3);
    }
    return 1;
}

// Pushes a value to the Lua stack.
void PushValue(lua_State *L, const int key) { lua_pushinteger(L, key); }
void PushValue(lua_State *L, const std::string &key)
{
    lua_pushstring(L, key.c_str());
}

// Reads the value with the given 'key' from the Lua dictionary and pushes it to
// the top of the stack.
template <typename T>
void GetValueFromLuaTable(lua_State *L, const T &key)
{
    PushValue(L, key);
    lua_rawget(L, -2);
}

// CHECK() that the topmost parameter on the Lua stack is a table.
void CheckTableIsAtTopOfStack(lua_State *L)
{
    CHECK(lua_istable(L, -1)) << "Topmost item on Lua stack is not a table!";
}

// Returns true if 'key' is in the table at the top of the Lua stack.
template <typename T>
bool HasKeyOfType(lua_State *L, const T &key)
{
    CheckTableIsAtTopOfStack(L);
    PushValue(L, key);
    lua_rawget(L, -2);
    const bool key_not_found = lua_isnil(L, -1);
    lua_pop(L, 1); // Pop the item again.
    return !key_not_found;
}

// Iterates over the integer keys of the table at the top of the stack of 'L•
// and pushes the values one by one. 'pop_value' is expected to pop a value and
// put them into a C++ container.
void GetArrayValues(lua_State *L, const std::function<void()> &pop_value)
{
    int idx = 1;
    while (true)
    {
        GetValueFromLuaTable(L, idx);
        if (lua_isnil(L, -1))
        {
            lua_pop(L, 1);
            break;
        }
        pop_value();
        ++idx;
    }
}

Lua_config_reader::Lua_config_reader(std::string file_name, std::unique_ptr<FileResolver> file_resolver)
: file_name_(file_name), L_(luaL_newstate()), file_resolver_(std::move(file_resolver))
{
    std::ifstream stream(file_name.c_str());
    std::string code = std::string((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());

    //auto code2 = file_resolver_->GetFileContentOrDie(file_name_);
    

    CHECK_NOTNULL(L_);
    SetDictionaryInRegistry(L_, this);

    luaL_openlibs(L_);

    //lua_register(L_, "choose", LuaChoose);
    lua_register(L_, "include", LuaInclude);
    //lua_register(L_, "read", LuaRead);

    CheckForLuaErrors(L_, luaL_loadstring(L_, code.c_str()));
    CheckForLuaErrors(L_, lua_pcall(L_, 0, 1, 0));
    CheckTableIsAtTopOfStack(L_);




}

int Lua_config_reader::LuaInclude(lua_State *L)
{
    CHECK_EQ(lua_gettop(L), 1);
    CHECK(lua_isstring(L, -1)) << "include takes a filename.";

    Lua_config_reader *Lua_config_reader = GetDictionaryFromRegistry(L);
    const std::string basename = lua_tostring(L, -1);
    const std::string filename = Lua_config_reader->file_resolver_->GetFullPathOrDie(basename);
    if (std::find(Lua_config_reader->included_files_.begin(),
    Lua_config_reader->included_files_.end(),
                  filename) != Lua_config_reader->included_files_.end())
    {
        std::string error_msg = "Tried to include " + filename +
                           " twice. Already included files in order of inclusion: ";
        for (const std::string &filename : Lua_config_reader->included_files_)
        {
            error_msg.append(filename);
            error_msg.append("\n");
        }
        LOG(FATAL) << error_msg;
    }
    Lua_config_reader->included_files_.push_back(filename);
    lua_pop(L, 1);
    CHECK_EQ(lua_gettop(L), 0);

    const std::string content = Lua_config_reader->file_resolver_->GetFileContentOrDie(basename);
    CheckForLuaErrors(L, luaL_loadbuffer(L, content.c_str(), content.size(), filename.c_str()));
    CheckForLuaErrors(L, lua_pcall(L, 0, LUA_MULTRET, 0));

    return lua_gettop(L);
}
