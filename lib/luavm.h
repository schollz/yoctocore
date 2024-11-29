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
#ifndef __linux__
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
#endif
//
#include "script.h"
#include "sequins.h"

lua_State *L;
int luaInit() {
  if (L != NULL) {
    return 0;
  }
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
  return 0;
}

/**
function update_env(i, code)
    envs[i] = new_env(code)
end
**/
int luaUpdateEnvironment(int index, const char *code) {
  lua_getglobal(L, "update_env");
  lua_pushinteger(L, index);
  lua_pushstring(L, code);
  if (lua_pcall(L, 2, 0, 0) != LUA_OK) {
    printf("Error: %s\n", lua_tostring(L, -1));
    lua_close(L);
    return 1;
  }
  return 0;
}

float luaRunMain(int index) {
  lua_getglobal(L, "env_main");
  lua_pushinteger(L, index);
  if (lua_pcall(L, 1, 1, 0) != LUA_OK) {
    printf("Error: %s\n", lua_tostring(L, -1));
    lua_close(L);
    return 0;
  }
  return lua_tonumber(L, -1);
}

int luaTest() {
  /**
  -- a = 10
-- function main()
--     a = a + 1
--     return a
-- end
  **/
  // add above code as update environment
  const char *code =
      "a = 10\n"
      "function main()\n"
      "    a = a + 1\n"
      "    return a\n"
      "end";
  luaUpdateEnvironment(0, code);

  // add another environment
  const char *code2 =
      "a = 20\n"
      "function main()\n"
      "    a = a + random_number()\n"
      "    return a\n"
      "end";
  luaUpdateEnvironment(1, code2);
  printf("Result: %f\n", luaRunMain(0));
  printf("Result: %f\n", luaRunMain(0));
  printf("Result: %f\n", luaRunMain(0));
  printf("Result: %f\n", luaRunMain(0));
  printf("Result: %f\n", luaRunMain(1));
  printf("Result: %f\n", luaRunMain(1));
  printf("Result: %f\n", luaRunMain(1));
  printf("Result: %f\n", luaRunMain(1));
  const char *code3 =
      "a = 1\n"
      "function main()\n"
      "    a = a + 2\n"
      "    return a\n"
      "end";
  luaUpdateEnvironment(1, code3);
  printf("Result: %f\n", luaRunMain(1));
  printf("Result: %f\n", luaRunMain(1));
  printf("Result: %f\n", luaRunMain(1));
  printf("Result: %f\n", luaRunMain(1));

  return 0;
}