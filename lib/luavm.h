// needed for lua
#include <errno.h>
#include <sys/types.h>

int _link(const char *oldpath, const char *newpath) {
  errno = ENOSYS;
  return -1;
}

int _unlink(const char *pathname) {
  errno = ENOSYS;
  return -1;
}

//
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
//
#include "script.h"
#include "sequins.h"

lua_State *L;
int runlua() {
  if (L == NULL) {
    L = luaL_newstate();  // Create a new Lua state
    luaL_openlibs(L);     // Open standard libraries

    // Load Lua script from embedded string
    if (luaL_loadbuffer(L, (const char *)lib_script_lua, lib_script_lua_len,
                        "embedded_script") != LUA_OK) {
      printf("Error loading Lua script: %s\n", lua_tostring(L, -1));
      lua_close(L);
      return 1;
    }

    // Execute the loaded Lua script
    if (lua_pcall(L, 0, 0, 0) != LUA_OK) {
      printf("Error executing Lua script: %s\n", lua_tostring(L, -1));
      lua_close(L);
      return 1;
    }

    // Load Lua script from embedded string
    if (luaL_loadbuffer(L, (const char *)lib_sequins_lua, lib_sequins_lua_len,
                        "embedded_script") != LUA_OK) {
      printf("Error loading Lua script: %s\n", lua_tostring(L, -1));
      lua_close(L);
      return 1;
    }

    // Execute the loaded Lua script
    if (lua_pcall(L, 0, 0, 0) != LUA_OK) {
      printf("Error executing Lua script: %s\n", lua_tostring(L, -1));
      lua_close(L);
      return 1;
    }

    // Define the new Lua function addone()
    const char *addone_code =
        "function addone()\n"
        "  local num = random_number()\n"
        "  return num + 0.1\n"
        "end\n"
        "abc = S{1, S{2, 3}, 3}";

    // Execute the new Lua function definition
    if (luaL_dostring(L, addone_code) != LUA_OK) {
      printf("Error defining addone function: %s\n", lua_tostring(L, -1));
      lua_close(L);
      return 1;
    }

    // Set up random number generator seed
    luaL_dostring(L, "math.randomseed(os.time())");
  }

  // Call the Lua function and get the result
  for (int i = 0; i < 10; i++) {
    lua_getglobal(L, "abc");                // Get the function from Lua
    if (lua_pcall(L, 0, 1, 0) != LUA_OK) {  // Call the function
      printf("Error: %s\n", lua_tostring(L, -1));
      lua_close(L);
      return 1;
    }

    double val = lua_tonumber(L, -1);  // Get the return value as a double
    lua_pop(L, 1);                     // Pop the result from the stack
    printf("random float: %.2f\n", val);

    if (i < 3) {
      lua_getglobal(L, "abc");       // Push 'abc' table onto the stack
      lua_getfield(L, -1, "reset");  // Get 'reset' method from 'abc'
      lua_pushvalue(L, -2);  // Push 'abc' table as the first argument (self)
      if (lua_pcall(L, 1, 0, 0) != LUA_OK) {  // Call 'reset' with 1 argument
        printf("Error calling abc:reset: %s\n", lua_tostring(L, -1));
        lua_pop(L, 1);  // Pop error message
      }
      lua_pop(L, 1);  // Pop 'abc' table from stack
    }
  }

  // lua_close(L);  // Close the Lua state
  return 0;
}