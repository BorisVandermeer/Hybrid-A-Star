/*********************************************************************
 * Author : BorisVandermeer
 * 
 * Discription ：
 *      A implement of Vehicle State.
 * 
*********************************************************************/

#include<Timer.h>

namespace Utils
{
    void Timer::Begin(){
        this->clear();
        start_ = std::chrono::system_clock::now();
    }

    void Timer::Toc(const std::string &task_name){
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double> use_time = end_ - start_;
        std::pair<std::string,double> tocdata = {task_name,use_time.count()};
        data_.push_back(tocdata);
    }

    void Timer::clear(){
        data_.clear();
    }

    double Timer::Stop() {
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double> use_time = end_ - start_;
        return use_time.count() * 1000.0;
    }


} // namespace Utils