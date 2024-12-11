// Copyright 2024 tml104
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ctime>
#include <iostream>
#include <chrono>
#include <vector>
#include <iomanip>
#include <sstream>

#include "SpdlogDef.hpp"
#include <spdlog/spdlog.h>

namespace Timer
{
    using MyClockType = std::chrono::time_point<std::chrono::high_resolution_clock>;

    class Timer
    {
    public:
        Timer() {Start();}

        void Start(){
            split_vec.clear();
            start_time_point = std::chrono::high_resolution_clock::now();
        }

        double GetSeconds(MyClockType c) const {
            auto duration =  std::chrono::duration_cast<std::chrono::seconds>(c - start_time_point);
            return duration.count();
        }

        void Split(std::string split_name){
            split_vec.emplace_back(split_name, std::chrono::high_resolution_clock::now());
        }

        void PrintTimes(){
            SPDLOG_INFO("===== {} =====", std::string("Timer"));

            for(auto p: split_vec){
                SPDLOG_INFO("{}: \t{:.6}", p.first, GetSeconds(p.second));
            }

            MyClockType now_time = std::chrono::high_resolution_clock::now();
            double now_sec = GetSeconds(now_time);
            SPDLOG_INFO("Total: {:.6}", now_sec);

            SPDLOG_INFO("===== {} =====", std::string("Timer End"));
        }

    private:
        MyClockType start_time_point;
        std::vector<std::pair<std::string, MyClockType>> split_vec;

    };

    std::string GetTime(){

        // 获取当前时间点
        auto now = std::chrono::system_clock::now();

        // 将当前时间点转换为系统时间（time_t）
        std::time_t t = std::chrono::system_clock::to_time_t(now);

        // 将time_t格式化为字符串
        std::tm tm = *std::localtime(&t);

        // 使用 std::put_time 输出格式化时间
        // std::cout << "当前时间是："
        //         << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << std::endl;

        // 使用 std::stringstream 存储格式化后的时间
        std::stringstream ss;
        ss << std::put_time(&tm, "%Y%m%d%H%M%S");

        // 返回字符串
        return ss.str();
    }

}