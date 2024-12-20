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
    bool is_bin;
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
        .add_option<bool>("-b", "--bin", "use bin", false)
        .parse(argc, argv);

	INPUT_ARGUMENTS input_args;
    input_args.file_path = args_parser.get_argument<std::string>("input_file_path");
    input_args.output_path = args_parser.get_option<std::string>("-o");
    input_args.is_bin = args_parser.get_option<bool>("-b");

    // set formatter
    // std::string log_name = "logs/default_log_" + Timer::GetTime() + ".log";
    auto file_path_split1 = YamadaMeshFixer::Utils::SplitStr(input_args.file_path, '/');
    auto file_path_split2 = YamadaMeshFixer::Utils::SplitStr(file_path_split1.back(), '.');

    std::string log_name = "logs/log_" + file_path_split2[0] + ".log"; // 现在根据文件名来

    auto file_logger = spdlog::basic_logger_mt("default_logger", log_name, true);
    spdlog::set_default_logger(file_logger);

    spdlog::set_pattern("[%H:%M:%S %z] [%^%L%$] [thread %t] [%s] [%@] [%!] %v");
    spdlog::set_level(spdlog::level::trace);

    if(input_args.is_bin){
        // load bin
        YamadaMeshFixer::BinInfo bin_info;
        bin_info.LoadFromBin(input_args.file_path);

        YamadaMeshFixer::MarkNum::GetInstance().LoadFromBinInfo(bin_info);
        
        for(auto solid: YamadaMeshFixer::MarkNum::GetInstance().solids){
            YamadaMeshFixer::StitchFixer2 stitchFixer(solid);
            stitchFixer.Start(true);

            stitchFixer.Test();
        }

        YamadaMeshFixer::MarkNum::GetInstance().ExportBin(input_args.output_path);
    }
    else{
        // load obj
        YamadaMeshFixer::ObjInfo obj_info;
        obj_info.LoadFromObj(input_args.file_path);

        YamadaMeshFixer::MarkNum::GetInstance().LoadFromObjInfo(obj_info);

        // test obj
        // YamadaMeshFixer::MarkNum::GetInstance().Test();

        for(auto solid: YamadaMeshFixer::MarkNum::GetInstance().solids){
            YamadaMeshFixer::StitchFixer2 stitchFixer(solid);
            stitchFixer.Start(true);

            stitchFixer.Test();
        }

        YamadaMeshFixer::MarkNum::GetInstance().ExportSolidsToOBJ(input_args.output_path);
    }

    return 0;
}