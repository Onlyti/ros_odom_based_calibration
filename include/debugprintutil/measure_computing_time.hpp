/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com, dudfhr3349@gmail.com
 * @file measure_computing_time.hpp
 * @brief header file to measure the computation time
 * @version 1.0
 * @date 21-04-2022
 * @bug No known bugs
 * @warning No warnings
 */

#ifndef __MEASURE_COMPUTING_TIME_HPP__
#define __MEASURE_COMPUTING_TIME_HPP__

#pragma once

#include <chrono>

class MeasureComputingTime
{
    // Constructor & Destructor

public:
    MeasureComputingTime( )
    :m_dProcessingTime_sec(0)
    {
        m_tpStartTime = std::chrono::steady_clock::now();
        m_tpEndTime = std::chrono::steady_clock::now();
    }
    ~MeasureComputingTime( ){}

    void Init( void ){}

    // Computational timer internal function

private:
    std::chrono::steady_clock::time_point m_tpStartTime;
    std::chrono::steady_clock::time_point m_tpEndTime;

public:
    void                            StartTimer( void )
    {
        m_tpStartTime = std::chrono::steady_clock::now();
    }
    double                          EndTimer( void )
    {
        m_tpEndTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> durationTime = m_tpEndTime - m_tpStartTime;
        m_dProcessingTime_sec = durationTime.count();
        
        return m_dProcessingTime_sec;
    }

    // Interface

private:
    double                          m_dProcessingTime_sec;

public:
    double                          GetProcessingTime_sec(void){ return m_dProcessingTime_sec; }

};
#endif