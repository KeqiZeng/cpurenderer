#include "model.h"
#include "image.h"
#include <algorithm>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

using Vec3d = Eigen::Vector3d;
using Vec2d = Eigen::Vector2d;

Model::Model(const std::string& directory, const std::string& filename, const std::string& texture_format)
{
    using path     = std::filesystem::path;
    path obj_model = path(directory) / path(filename);
    name_          = obj_model.filename().string();
    std::ifstream in;
    in.open(obj_model);
    if (!in.is_open()) {
        std::cerr << std::format("Failed to open file: {}\n", obj_model.c_str());
        return;
    }

    bounding_box_min_ = Vec3d(
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max()
    );
    bounding_box_max_ = Vec3d(
        -std::numeric_limits<double>::max(),
        -std::numeric_limits<double>::max(),
        -std::numeric_limits<double>::max()
    );

    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (line.substr(0, 2) == "v ") {
            iss >> trash; // discard 'v'
            double x, y, z;
            iss >> x >> y >> z;
            vertices_.emplace_back(x, y, z);
            bounding_box_min_[0] = std::min(bounding_box_min_[0], x);
            bounding_box_min_[1] = std::min(bounding_box_min_[1], y);
            bounding_box_min_[2] = std::min(bounding_box_min_[2], z);
            bounding_box_max_[0] = std::max(bounding_box_max_[0], x);
            bounding_box_max_[1] = std::max(bounding_box_max_[1], y);
            bounding_box_max_[2] = std::max(bounding_box_max_[2], z);
        }
        else if (line.starts_with("vn ")) {
            iss >> trash >> trash; // discard 'vn'
            double x, y, z;
            iss >> x >> y >> z;
            auto n = Vec3d(x, y, z);
            n.normalize();
            normals_.push_back(n);
        }
        else if (line.starts_with("vt ")) {
            iss >> trash >> trash; // discard 'vt'
            double u, v;
            iss >> u >> v;
            uvs_.emplace_back(u, v);
        }
        else if (line.starts_with("f ")) {
            if (vertices_.empty() || normals_.empty() || uvs_.empty()) {
                std::cerr << "Error: the obj file is supposed to contain vertices, normals and texture coordinates"
                          << std::endl;
                return;
            }
            iss >> trash; // discard 'f'
            int f, uv, n;
            int cnt = 0;
            Face face;
            while (iss >> f >> trash >> uv >> trash >> n) {
                face.v_idx_[cnt]  = --f;
                face.n_idx_[cnt]  = --n;
                face.uv_idx_[cnt] = --uv;
                cnt++;
            }
            if (cnt != 3) {
                std::cerr << "Error: the obj file is supposed to be triangulated" << std::endl;
                return;
            }
            faces_.push_back(face);
        }
    }
    bounding_box_center_ = (bounding_box_min_ + bounding_box_max_) / 2.0;

    std::filesystem::path diffuse_map  = formatTexturePath(directory, filename, "_diff", texture_format);
    diffuse_map_                       = std::make_unique<RGBImage>(diffuse_map.c_str());
    std::filesystem::path specular_map = formatTexturePath(directory, filename, "_spec", texture_format);
    specular_map_                      = std::make_unique<GrayScaleImage>(specular_map.c_str());

    try {
        std::filesystem::path normal_map = formatTexturePath(directory, filename, "_nm_tangent", texture_format);
        normal_map_                      = std::make_unique<RGBImage>(normal_map.c_str());
    }
    catch (const std::exception& err) {
        normal_map_ = nullptr;
    }
}

const std::string& Model::getName() const
{
    return name_;
}

const Vec3d& Model::getVertex(int idx) const
{
    return vertices_[idx];
}

const Vec3d& Model::getNormal(int idx) const
{
    return normals_[idx];
}

const Vec2d& Model::getUV(int idx) const
{
    return uvs_[idx];
}

const Face& Model::getFace(int idx) const
{
    return faces_[idx];
}

int Model::getFaceCount() const
{
    return static_cast<int>(faces_.size());
}

const Vec3d& Model::getBoundingBoxMin() const
{
    return bounding_box_min_;
}

const Vec3d& Model::getBoundingBoxMax() const
{
    return bounding_box_max_;
}

const Vec3d& Model::getBoundingBoxCenter() const
{
    return bounding_box_center_;
}

const std::unique_ptr<RGBImage>& Model::getDiffuseMap() const
{
    return diffuse_map_;
}

const std::unique_ptr<RGBImage>& Model::getNormalMap() const
{
    return normal_map_;
}

const std::unique_ptr<GrayScaleImage>& Model::getSpecularMap() const
{
    return specular_map_;
}

std::string Model::formatTexturePath(
    const std::string& directory,
    const std::string& filename,
    const std::string& texture_suffix,
    const std::string& texture_format
)
{
    size_t dot = filename.find_last_of(".");
    if (dot == std::string::npos)
        return "";
    return directory + "/" + filename.substr(0, dot) + texture_suffix + "." + texture_format;
}

ModelList::ModelList(const std::string& directory, const std::string& texture_format)
{
    namespace fs = std::filesystem;
    auto path    = fs::path(directory);
    if (!fs::exists(path) || !fs::is_directory(path)) {
        std::cerr << std::format("Failed to open directory: {}\n", directory);
    }

    try {
        auto options = fs::directory_options::skip_permission_denied;
        for (const auto& entry : fs::directory_iterator(path, options)) {
            if (entry.is_regular_file()) {
                std::string ext = entry.path().extension().string();
                std::ranges::transform(ext, ext.begin(), ::tolower);
                if (ext == ".obj") {
                    models_.push_back(
                        std::make_shared<Model>(directory, entry.path().filename().string(), texture_format)
                    );
                }
            }
        }
    }
    catch (const std::exception& err) {
        std::cerr << std::format("Failed to traverse the directory: {}\n", directory);
        throw err;
    }
}

std::vector<std::shared_ptr<Model>>& ModelList::getModels()
{
    return models_;
}
