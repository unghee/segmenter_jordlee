/**
 * @file ConfigFile.hh
 * @author Michael Zillich
 * @date 2006
 * @version 0.1
 * @brief Work with configurations from a config file.
 **/

#ifndef Z_CONFIG_FILE_HH
#define Z_CONFIG_FILE_HH

#include <string>
#include <fstream>
// #include "Namespace.hh"

namespace Z
{

using namespace std;

/**
 * @brief Class Work with configurations from a config file.
 */
class ConfigFile
{
private:
  fstream file;														///< File stream
  int line_cnt;														///< Line counter

private:
  void RemoveEOL(string &str);
  bool IsComment(string &str);

public:
  ConfigFile(const string &name);
  ~ConfigFile();
  bool GetLine(string &str);
  int GetLineCnt() {return line_cnt;}			///< Get line counter number
};

bool GetWord(const string &str, string &word, string::size_type &pos);

}

#endif

