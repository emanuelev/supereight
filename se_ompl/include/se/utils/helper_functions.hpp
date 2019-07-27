//
// Created by anna on 01/05/19.
//

#ifndef EXPLORATION_WS_HELPER_FUNCTIONS_HPP
#define EXPLORATION_WS_HELPER_FUNCTIONS_HPP

#include <time.h>
#include <stdio.h>
#include <string>

namespace se {
namespace exploration {

/**
 * @brief returns timestamped filename
 */
std::string getTimeStampedFilename() {

  char timestamp[6];
  time_t rawtime = time(NULL);
  tm *now = localtime(&rawtime);

  strftime(timestamp, 6, "%H%M%S", now);
  std::string filename = timestamp;

  return "logs/" + filename + "_rrt_output.txt";

}

}//namespace exploration
}
#endif //EXPLORATION_WS_HELPER_FUNCTIONS_HPP
