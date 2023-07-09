/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A implement of Vehicle State.
 * 
*********************************************************************/

#ifndef _CPU_TIMER_H_
#define _CPU_TIMER_H_

#include <vector>
#include <string>
#include <chrono>
#include <unordered_map>

namespace Utils
{
    class Timer{
    public:
        Timer(){};
        void Begin();
        void Toc(const std::string &task_name);
        void clear();
        double Stop(); 

        std::vector<std::pair<std::string,double>> getData;


    private:
        std::chrono::time_point<std::chrono::system_clock> start_, end_;
        std::vector<std::pair<std::string,double>> data_;

    };
} // namespace Utils



#endif