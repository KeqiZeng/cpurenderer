#pragma once

#include <Eigen/Dense>
#include <string>
#include <string_view>

class Args
{
    using Vec3d = Eigen::Vector3d;

  public:
    static constexpr double FOV_Y            = 60.0;
    static constexpr double ASPECT_RATIO     = 1.0;
    static constexpr double NEAR             = -0.1;
    static constexpr double FAR              = -50.0;
    static constexpr int IMG_WIDTH           = 800;
    static constexpr int IMG_HEIGHT          = 800;
    static constexpr double LIGHT_INTENSITY  = 1.6;
    static constexpr double KA               = 0.1;
    static constexpr double KD               = 0.5;
    static constexpr double KS               = 0.5;
    static constexpr char const* OUTPUT_FILE = "output.png";

    Args(int argc, char** argv, std::string_view program_name);

    std::string input_dir_;
    std::string texture_format_;
    Vec3d camera_position_;
    Vec3d camera_lookat_;
    Vec3d camera_up_;
    Vec3d light_position_;
};
