#include "timer.h"


Timer::Timer(const std::string &name) :
    m_name(name)
{
    m_start = std::chrono::system_clock::now();
    std::cout << "=== START: " << m_name << std::endl;
}

Timer::~Timer()
{
    m_end = std::chrono::system_clock::now();
    const std::chrono::duration<double> elapsed_seconds = m_end-m_start;

    std::cout << "=== FINISH: " << m_name << ": "
            << elapsed_seconds.count()*1000 << " ms" << std::endl;
}
