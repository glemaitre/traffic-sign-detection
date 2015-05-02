#pragma once

#include <chrono>
#include <iostream>

//TODO: Print the output of this class only when verbose mode, make verbose cmake option flag

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
