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
#include <spdlog/sinks/basic_file_sink.h>

#include "argparser.hpp"
#include "Timer.hpp"

#include "YamadaMeshFixer.hpp"

struct INPUT_ARGUMENTS{
    std::string file_path;
    std::string output_path;
};

int main(int argc, char const *argv[]){

    // parse args
    // TOOD: 到时候可能要改
	auto args_parser = util::argparser("YamadaMeshFixer by TML104");
	args_parser.set_program_name("YamadaMeshFixer")
		.add_help_option()
		.use_color_error()
		.add_argument<std::string>("input_file_path", "model path")
        .add_option<std::string>("-o", "--output", "output obj path", "./output_obj.obj")
        .parse(argc, argv);

	INPUT_ARGUMENTS input_args;
    input_args.file_path = args_parser.get_argument<std::string>("input_file_path");
    input_args.output_path = args_parser.get_option<std::string>("-o");

    // set formatter
    auto file_logger = spdlog::basic_logger_mt("default_logger", "logs/default_log.txt");
    spdlog::set_default_logger(file_logger);

    spdlog::set_pattern("[%H:%M:%S %z] [%^%L%$] [thread %t] [%s] [%@] [%!] %v");
    spdlog::set_level(spdlog::level::trace);

    // SPDLOG_DEBUG("Test: {}", 789);
    // SPDLOG_TRACE("Test: {}", 456);
    // SPDLOG_INFO("Test: {}", 123);

    // TODO: load obj
    YamadaMeshFixer::ObjInfo obj_info;
    obj_info.LoadFromObj(input_args.file_path);

    YamadaMeshFixer::MarkNum::GetInstance().LoadFromObjInfo(obj_info);

    // TODO: test obj
    YamadaMeshFixer::MarkNum::GetInstance().Test();

    // Stitch
    for(auto solid: YamadaMeshFixer::MarkNum::GetInstance().solids){
        YamadaMeshFixer::StitchFixer stitchFixer(solid);
        stitchFixer.Start(true, false);
    }

    YamadaMeshFixer::MarkNum::GetInstance().ExportSolidsToOBJ(input_args.output_path);

    return 0;
}