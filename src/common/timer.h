#pragma once

#include <chrono>
#include <iostream>

/**
 * @brief The Timer class is a RAII to measure time, from instantiation to end of scope
 */
class Timer
{

public:

    Timer(const std::string & name);

    ~Timer();

private:

    std::chrono::time_point<std::chrono::system_clock> m_start, m_end;
    std::string m_name;

};
