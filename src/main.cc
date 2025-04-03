#include "args.h"
#include "camera.h"
#include "model.h"
#include "painter.h"
#include <cstddef>
#include <exception>
#include <format>
#include <iostream>

int main(int argc, char** argv)
{
    auto args = Args(argc, argv, "cpurenderer");

    try {
        auto model_list = ModelList(args.input_dir_, args.texture_format_);
        auto painter    = Painter(
            Camera(args.camera_position_, args.camera_lookat_, args.camera_up_),
            args.light_position_,
            Args::LIGHT_INTENSITY
        );
        auto shader = PhongShader();

        painter.generateShadowBuffer(model_list);
        for (const auto& model : model_list.getModels()) {
            if (painter.viewFrustumCulling(model)) {
                shader.setModel(model);
                for (int i = 0; i < model->getFaceCount(); ++i) {
                    if (painter.backFaceCulling(model, i)) {
                        painter.paint(model, i, shader);
                    }
                }
            }
            else {
                std::cerr << std::format("{} is out of view", model->getName()) << std::endl;
            }
        }
        painter.getImage().saveToFile(Args::OUTPUT_FILE);
    }
    catch (const std::exception& err) {
        std::cerr << err.what() << std::endl;
        return 1;
    }
    return 0;
}
