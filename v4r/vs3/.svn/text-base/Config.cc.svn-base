/**
 * @file Config.cc
 * @author Michael Zillich
 * @date 2006
 * @version 0.1
 * @brief Work with configurations from a config file.\n
 * Implementation of the higher-level functions.
 **/

#include "ConfigFile.hh"
#include "Config.hh"

namespace Z
{

/**
 * @brief Constructor of Config.
 * @param filename Name of configuration file.
 */
Config::Config(const string &filename)
{
  Load(filename);
}

/**
 * @brief Load configuration from a file with "filename".\n
 * @param filename Name of configuration file.
 */
void Config::Load(const string &filename)
{
  ConfigFile file(filename);
  string line, name, value;
  string::size_type pos;
  while(file.GetLine(line))
  {
    pos = 0;
    GetWord(line, name, pos);
    GetWord(line, value, pos);
    items[name] = value;
  }
}

}

