#pragma once
#include "image.h"
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <string>
#include <vector>

struct Face
{
    std::array<int, 3> v_idx_;
    std::array<int, 3> n_idx_;
    std::array<int, 3> uv_idx_;
};

class Model
{
    using Vec3d = Eigen::Vector3d;
    using Vec2d = Eigen::Vector2d;

  public:
    Model(const std::string& directory, const std::string& model_name, const std::string& texture_format);
    Model(const Model&)            = delete;
    Model(Model&&)                 = default;
    Model& operator=(const Model&) = delete;
    Model& operator=(Model&&)      = default;

    ~Model() = default;

    [[nodiscard]] const std::string& getName() const;
    [[nodiscard]] const Vec3d& getVertex(int idx) const;
    [[nodiscard]] const Vec3d& getNormal(int idx) const;
    [[nodiscard]] const Vec2d& getUV(int idx) const;
    [[nodiscard]] const Face& getFace(int idx) const;
    [[nodiscard]] int getFaceCount() const;
    [[nodiscard]] const Vec3d& getBoundingBoxMin() const;
    [[nodiscard]] const Vec3d& getBoundingBoxMax() const;
    [[nodiscard]] const Vec3d& getBoundingBoxCenter() const;
    [[nodiscard]] const std::unique_ptr<RGBImage>& getDiffuseMap() const;
    [[nodiscard]] const std::unique_ptr<RGBImage>& getNormalMap() const;
    [[nodiscard]] const std::unique_ptr<GrayScaleImage>& getSpecularMap() const;

  private:
    std::string name_;

    std::vector<Vec3d> vertices_;
    std::vector<Vec3d> normals_;
    std::vector<Vec2d> uvs_;
    std::vector<Face> faces_;

    Vec3d bounding_box_min_;
    Vec3d bounding_box_max_;
    Vec3d bounding_box_center_;

    std::unique_ptr<RGBImage> diffuse_map_;
    std::unique_ptr<RGBImage> normal_map_;
    std::unique_ptr<GrayScaleImage> specular_map_;

    std::string formatTexturePath(
        const std::string& directory,
        const std::string& filename,
        const std::string& texture_suffix,
        const std::string& texture_format
    );
};

class ModelList
{
  public:
    ModelList(const std::string& directory, const std::string& texture_format);
    std::vector<std::shared_ptr<Model>>& getModels();

  private:
    std::vector<std::shared_ptr<Model>> models_;
};
