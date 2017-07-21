/**
 * @file Config.hh
 * @author Michael Zillich
 * @date 2006
 * @version 0.1
 * @brief Work with configurations from a config file.\n
 * Implementation of the higher-level functions.
 **/

#ifndef Z_CONFIG_HH
#define Z_CONFIG_HH

#include <stdlib.h>
#include <string>
#include <map>
// #include "Namespace.hh"

namespace Z
{
	using namespace std;

/**
 * @brief Class Config
 */
class Config
{
public:
  map<string, string> items;					///< Items in the configuration.

public:
  Config() {}
  Config(const string &filename);
  void Load(const string &filename);
  void AddItem(const string &name, const string &value) {items[name] = value;}		///< Adds item to the configuration.
  string GetValueString(const string &name) {return items[name];}									///< Returns value as string.
  int GetValueInt(const string &name) {return atoi(items[name].c_str());}					///< Returns value as integer.
  double GetValueDouble(const string &name) {return atof(items[name].c_str());}		///< Returns value as double.
};

}

#endif

