#include "args.h"
#include "argparse.hpp"
#include <vector>

Args::Args(int argc, char** argv, std::string_view program_name)
{
    argparse::ArgumentParser program("cpurenderer");
    program.add_argument("input_dir").help("the directory of the input models").required();
    program.add_argument("--texture_format").help("the file format of the textures").required().default_value("tga");
    program.add_argument("--camera_position")
        .help("the position of the camera")
        .nargs(3)
        .default_value(std::vector<double>{0.5, 0.6, 1.6})
        .scan<'g', double>();
    program.add_argument("--camera_lookat")
        .help("the point where the camera looks at")
        .nargs(3)
        .default_value(std::vector<double>{0.0, 0.0, 0.0})
        .scan<'g', double>();
    program.add_argument("--camera_up")
        .help("the up direction of the camera")
        .nargs(3)
        .default_value(std::vector<double>{0.0, 1.0, 0.0})
        .scan<'g', double>();
    program.add_argument("--light_position")
        .help("the position of the light")
        .nargs(3)
        .default_value(std::vector<double>{1.5, 2.5, 2.5})
        .scan<'g', double>();

    program.parse_args(argc, argv);

    input_dir_      = program.get<std::string>("input_dir");
    texture_format_ = program.get<std::string>("--texture_format");

    auto camera_position = program.get<std::vector<double>>("--camera_position");
    auto camera_lookat   = program.get<std::vector<double>>("--camera_lookat");
    auto camera_up       = program.get<std::vector<double>>("--camera_up");
    auto light_position  = program.get<std::vector<double>>("--light_position");

    camera_position_ = Vec3d(camera_position[0], camera_position[1], camera_position[2]);
    camera_lookat_   = Vec3d(camera_lookat[0], camera_lookat[1], camera_lookat[2]);
    camera_up_       = Vec3d(camera_up[0], camera_up[1], camera_up[2]);
    light_position_  = Vec3d(light_position[0], light_position[1], light_position[2]);
}
