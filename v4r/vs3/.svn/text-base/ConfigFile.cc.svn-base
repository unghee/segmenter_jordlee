/**
 * @file ConfigFile.cc
 * @author Michael Zillich
 * @date 2006
 * @version 0.1
 * @brief Work with configurations from a config file.
 **/

#include <stdexcept>
#include "ConfigFile.hh"

namespace Z
{

/**
 * @brief Constructor of class ConfigFile.
 * @param name Name of the config file.
 */
ConfigFile::ConfigFile(const string &name)
{
  line_cnt = 0;
  file.open(name.c_str(), ios::in);
  if(!file)
	{
		char buffer [150];
		sprintf(buffer, "ConfigFile::ConfigFile: Failed to open file '%s'", name.c_str());
    throw std::runtime_error(buffer);
	}
}

/**
 * @brief Destructor of class ConfigFile.
 */
ConfigFile::~ConfigFile()
{
  file.close();
}

/**
 * @brief Reads the next line of a file into the given string ignoring comment lines.\n
 * Returns 'true' if the line was filled. 'false' otherwise (i.e. if EOF was\n
 * reached). The newline character is NOT appended.\n
 * If the file was not opened readable then an exception is thrown.
 * @param str String
 */
bool ConfigFile::GetLine(string &str)
{
  bool success = false;
  while(file && !success)
  {
    // get the next line
    str.erase();
    getline(file, str);
    line_cnt++;
    RemoveEOL(str);
    if(!IsComment(str))
      success = true;
  }
  return success;
}

/**
 * @brief Remove end of line.\n
 * Note that end of line may be `\n` (newline, unix style)\n
 * or '\r' (carriage return, dos style, wrong! very very wrong!!)\n
 * We have to check for both.
 * @param str String
 */
void ConfigFile::RemoveEOL(string &str)
{
  string::size_type p = str.find_last_of("\n\r");
  if(p != string::npos)
    str.erase(p, 1);
}

/**
 * @brief Returns true if the line is a comment line.\n
 * Comment lines are empty lines or start with "#".
 * @param str String
 */
bool ConfigFile::IsComment(string &str)
{
  // skip leading whitespaces
  string::size_type p = str.find_first_not_of(" \f\n\r\t\v");
  if(p == string::npos)
    return true;
  if(str[p] == '#')
    return true;
  return false;
}


/**
 * @brief Get the next word from a string, starting at pos.\n
 * Words are separated by whitespaces (as in isspace(3)). The string must\n
 * contain only a single line.\n
 * pos contains the position after the word or string::npos if at end of string.\n
 * Returns true if a word was found, false otherwise.
 * @param str String to read from.
 * @param word Word
 * @param pos Position
 * @return True for success.
 */
bool GetWord(const string &str, string &word, string::size_type &pos)
{
  // skip leading whitespaces
  string::size_type s = str.find_first_not_of(" \f\n\r\t\v", pos);
  if(s != string::npos)
  {
    string::size_type e = str.find_first_of(" \f\n\r\t\v", s);
    if(e != string::npos)
      word = str.substr(s, e - s);
    else
      word = str.substr(s);
    pos = e;
    return true;
  }
  else
  {
    pos = s;
    return false;
  }
}

}

