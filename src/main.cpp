#include <iostream>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include <spdlog/spdlog.h>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "argparser.hpp"
#include "Timer.hpp"

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
		.add_argument<std::string>("input_obj_file", "stl model path")
        .parse(argc, argv);

	INPUT_ARGUMENTS input_args;
    input_args.file_path = args_parser.get_argument<std::string>("input_obj_file");

    // set formatter
    spdlog::set_pattern("[%H:%M:%S %z] [%n] [%^%L%$] [thread %t] [%s] [%@] %v");

    SPDLOG_DEBUG("Test: {}", 314159);


    return 0;
}