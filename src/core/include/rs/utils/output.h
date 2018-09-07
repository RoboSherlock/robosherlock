/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OUTPUT_H_
#define OUTPUT_H_

#include <iostream>
#include <string.h>
#include <ros/console.h>

#define NO_COLOR        "\033[0m"

#define FG_BLACK        "\033[30m"
#define FG_RED          "\033[31m"
#define FG_GREEN        "\033[32m"
#define FG_YELLOW       "\033[33m"
#define FG_BLUE         "\033[34m"
#define FG_MAGENTA      "\033[35m"
#define FG_CYAN         "\033[36m"
#define FG_LIGHTGREY    "\033[37m"
#define FG_GREY         "\033[90m"
#define FG_LIGHTRED     "\033[91m"
#define FG_LIGHTGREEN   "\033[92m"
#define FG_LIGHTYELLOW  "\033[93m"
#define FG_LIGHTBLUE    "\033[94m"
#define FG_LIGHTMAGENTA "\033[95m"
#define FG_LIGHTCYAN    "\033[96m"
#define FG_WHITE        "\033[97m"

#define BG_BLACK        "\033[40m"
#define BG_RED          "\033[41m"
#define BG_GREEN        "\033[42m"
#define BG_YELLOW       "\033[43m"
#define BG_BLUE         "\033[44m"
#define BG_MAGENTA      "\033[45m"
#define BG_CYAN         "\033[46m"
#define BG_LIGHTGREY    "\033[47m"
#define BG_GREY         "\033[100m"
#define BG_LIGHTRED     "\033[101m"
#define BG_LIGHTGREEN   "\033[102m"
#define BG_LIGHTYELLOW  "\033[103m"
#define BG_LIGHTBLUE    "\033[104m"
#define BG_LIGHTMAGENTA "\033[105m"
#define BG_LIGHTCYAN    "\033[106m"
#define BG_WHITE        "\033[107m"

#define OUT_LEVEL_NOOUT 0
#define OUT_LEVEL_ERROR 1
#define OUT_LEVEL_INFO  2
#define OUT_LEVEL_DEBUG 3

#ifndef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_INFO
#endif

#define OUT_STD_STREAM std::cout
#define OUT_ERR_STREAM std::cerr

#define OUT_FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define OUT_AUX_INT(FILE_COLOR, MSG_COLOR, LEVEL, STREAM, MSG, FILE, LINE, FUNCTION) LEVEL <= OUT_LEVEL && STREAM << FILE_COLOR << FILE << NO_COLOR "(" FG_CYAN << LINE << NO_COLOR ")[" FG_YELLOW << FUNCTION << NO_COLOR "] " MSG_COLOR << MSG << NO_COLOR << std::endl << std::flush
#define OUT_AUX(FILE_COLOR, MSG_COLOR, LEVEL, STREAM, MSG) OUT_AUX_INT(FILE_COLOR, MSG_COLOR, LEVEL, STREAM, MSG, OUT_FILENAME, __LINE__, __FUNCTION__)

#define outDebug(msg) ROS_DEBUG_STREAM(FG_BLUE << OUT_FILENAME << NO_COLOR "(" FG_CYAN << __LINE__ << NO_COLOR ")[" FG_YELLOW << __FUNCTION__ << NO_COLOR "] " NO_COLOR << msg << NO_COLOR )
#define outInfo(msg) ROS_INFO_STREAM(FG_GREEN << OUT_FILENAME << NO_COLOR "(" FG_CYAN << __LINE__ << NO_COLOR ")[" FG_YELLOW << __FUNCTION__ << NO_COLOR "] " NO_COLOR << msg << NO_COLOR )
#define outWarn(msg) ROS_INFO_STREAM(FG_YELLOW << OUT_FILENAME << FG_YELLOW "(" FG_CYAN << __LINE__ << NO_COLOR ")[" FG_YELLOW << __FUNCTION__ << NO_COLOR "] " FG_YELLOW << msg << FG_YELLOW )
#define outError(msg) ROS_ERROR_STREAM(FG_RED << OUT_FILENAME << NO_COLOR "(" FG_RED << __LINE__ << FG_RED ")[" FG_RED << __FUNCTION__ << FG_RED "] " FG_RED << msg << FG_RED )


#define outAssert(expr,msg) if (!(expr)) OUT_AUX(FG_MAGENTA, FG_MAGENTA, OUT_LEVEL_ERROR, OUT_ERR_STREAM, msg)

#endif /* OUTPUT_H_ */
