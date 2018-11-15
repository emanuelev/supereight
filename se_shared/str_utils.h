#ifndef STR_UTILS_H
#define STR_UTILS_H

#include <sstream>
#include <string>
#include <vector>

/**
 * Split a string into a vector of substrings based on a delimiter.
 *
 * @note The contents of `elems` are not deleted, substrings are just pushed
 * back.
 *
 * \param[in] s The string to split.
 * \param[in] delim The delimiter to use.
 * \param[out] elems The vector that will contain the substrings.
 * \return A reference to the vector containing the substrings.
 */
static std::vector<std::string>& splitString(const std::string&        s,
                                             char                      delim,
                                             std::vector<std::string>& elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

/**
 * Split a string into a vector of substrings based on a delimiter.
 *
 * @note The contents of `elems` are not deleted, substrings are just pushed
 * back.
 *
 * \param[in] s The string to split.
 * \param[in] delim The delimiter to use.
 * \return The vector containing the substrings.
 */
static std::vector<std::string> splitString(const std::string& s, char delim) {
  std::vector<std::string> elems;
  splitString(s, delim, elems);
  return elems;
}

#endif
