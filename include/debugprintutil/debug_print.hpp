/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author dudfhe3349@gmail.com pauljiwon96@gmail.com
 * @file dubug_print.hpp
 * @brief debug printer
 * @version 1.0
 * @date 13-02-2022
 * @bug No known bugs
 * @warning No warnings
 */

#ifndef __DEBUG_PRINT_HPP__
#define __DEBUG_PRINT_HPP__

#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include "measure_computing_time.hpp"

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

enum Color{ RESET,
            BLACK,
            RED,
            GREEN,
            YELLOW,
            BLUE,
            MAGENTA,
            CYAN,
            WHITE,
            BOLDBLACK,
            BOLDRED,
            BOLDGREEN,
            BOLDYELLOW,
            BOLDBLUE,
            BOLDMAGENTA,
            BOLDCYAN,
            BOLDWHITE};

static const char *enum_str[] ={"\033[0m",/* Reset */
                                "\033[30m",/* Black */
                                "\033[31m",/* Red */
                                "\033[32m",/* Green */
                                "\033[33m",/* Yellow */
                                "\033[34m",/* Blue */
                                "\033[35m",/* Magenta */
                                "\033[36m",/* Cyan */
                                "\033[37m",/* White */
                                "\033[1m\033[30m",/* Bold Black */
                                "\033[1m\033[31m",/* Bold Red */
                                "\033[1m\033[32m",/* Bold Green */
                                "\033[1m\033[33m",/* Bold Yellow */
                                "\033[1m\033[34m",/* Bold Blue */
                                "\033[1m\033[35m",/* Bold Magenta */
                                "\033[1m\033[36m",/* Bold Cyan */
                                "\033[1m\033[37m"/* Bold White */};



template <typename TYPE>
inline void DebugPrintInfo(std::string debug_info,TYPE value, bool flag=true, Color color=RESET)
{
    if(flag)
    {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout<< enum_str[color] << "[Info]" << "[" << time  <<"] " << debug_info << " " <<  value << enum_str[RESET] << std::endl;
    }
}

template <typename TYPE>
inline void DebugPrintWarn(std::string debug_info,TYPE value, bool flag=true, Color color=YELLOW)
{
    if(flag)
    {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[Warn]" << "[" << time  <<"] " << debug_info << " " <<  value << enum_str[RESET] << std::endl;
    }
}

template <typename TYPE>
inline void DebugPrintError(std::string debug_info,TYPE value, bool flag=true, Color color=BOLDRED)
{
    if(flag)
    {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[Error]" << "[" << time  <<"] " << debug_info << " " <<  value << enum_str[RESET] << std::endl;
    }
}

inline void DebugPrintInfo(std::string debug_info, bool flag=true, Color color=RESET)
{
    if(flag)
    {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[Info]" << "[" << time <<"] " << debug_info << enum_str[RESET] << std::endl;
    }
}

inline void DebugPrintWarn(std::string debug_info, bool flag=true, Color color=YELLOW)
{
    if(flag)
    {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[Warn]" << "[" << time <<"] " << debug_info <<  enum_str[RESET] << std::endl;
    }
}

inline void DebugPrintError(std::string debug_info, bool flag=true, Color color=BOLDRED)
{
    if(flag)
    {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[Error]" << "[" << time <<"] " << debug_info <<  enum_str[RESET] << std::endl;
    }
}

inline void DebugPrintInfoTime(std::string debug_info,MeasureComputingTime &measure_time, bool flag=true, Color color=RESET)
{
    if(flag)
    {
        measure_time.EndTimer();
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[Info]" << "[" << time <<"] " << "[Time diff: "<<std::to_string(measure_time.GetProcessingTime_sec())<<"(s)] "  << debug_info << enum_str[RESET] << std::endl;
        measure_time.StartTimer();
    }
}

inline void DebugPrintWarnTime(std::string debug_info,MeasureComputingTime &measure_time, bool flag=true, Color color=YELLOW)
{
    if(flag)
    {
        measure_time.EndTimer();
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[Warm]" << "[" << time <<"] " << "[Time diff: "<<std::to_string(measure_time.GetProcessingTime_sec())<<"(s)] " << debug_info << enum_str[RESET] << std::endl;
        measure_time.StartTimer();
    }
}

inline void DebugPrintErrorTime(std::string debug_info,MeasureComputingTime &measure_time, bool flag=true, Color color=BOLDRED)
{
    if(flag)
    {
        measure_time.EndTimer();
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[Error]" << "[" << time <<"] " << "[Time diff: "<<std::to_string(measure_time.GetProcessingTime_sec())<<"(s)] "  << debug_info << enum_str[RESET] << std::endl;
        measure_time.StartTimer();
    }
}

#endif