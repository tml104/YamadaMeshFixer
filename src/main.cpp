#include <iostream>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include "SpdlogDef.hpp"
#include <spdlog/spdlog.h>

#include "argparser.hpp"
#include "Timer.hpp"

#include "YamadaMeshFixer.hpp"

struct INPUT_ARGUMENTS{
    std::string file_path;
};

int main(int argc, char const *argv[]){

    // parse args
    // TOOD: 到时候可能要改
	auto args_parser = util::argparser("YamadaMeshFixer by TML104");
	args_parser.set_program_name("YamadaMeshFixer")
		.add_help_option()
		.use_color_error()
		.add_argument<std::string>("input_file_path", "model path")
        .parse(argc, argv);

	INPUT_ARGUMENTS input_args;
    input_args.file_path = args_parser.get_argument<std::string>("input_file_path");

    // set formatter
    spdlog::set_pattern("[%H:%M:%S %z] [%n] [%^%L%$] [thread %t] [%s] [%@] %v");
    spdlog::set_level(spdlog::level::trace);

    SPDLOG_DEBUG("Test: {}", 789);
    SPDLOG_TRACE("Test: {}", 456);
    SPDLOG_INFO("Test: {}", 123);

    // TODO: load obj
    YamadaMeshFixer::ObjInfo obj_info;
    obj_info.LoadFromObj(input_args.file_path);

    // YamadaMeshFixer::MarkNum m;
    // m.LoadFromObjInfo(obj_info);

    // TODO: test obj
    // m.Test();


    return 0;
}