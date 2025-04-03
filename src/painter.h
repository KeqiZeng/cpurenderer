#pragma once

#include "args.h"
#include "camera.h"
#include "image.h"
#include "model.h"
#include <Eigen/Dense>
#include <array>
#include <memory>
#include <vector>

class Painter;

class Shader
{
  public:
    using Vec2d = Eigen::Vector2d;
    using Vec3d = Eigen::Vector3d;
    using Vec4d = Eigen::Vector4d;

    Shader()                                = default;
    Shader(const Shader& shader)            = default;
    Shader(Shader&& shader)                 = default;
    Shader& operator=(const Shader& shader) = default;
    Shader& operator=(Shader&& shader)      = default;
    virtual ~Shader()                       = default;

    [[nodiscard]] virtual Vec4d vertex(int face_idx, int vertex_idx, const Painter& painter)                      = 0;
    [[nodiscard]] virtual std::unique_ptr<Color> fragment(const Vec3d& bary_coords, const Painter& painter) const = 0;

    void setModel(std::shared_ptr<Model> model);

  protected:
    std::shared_ptr<Model> model_;
};

class PhongShader : public Shader
{
  public:
    PhongShader() = default;
    [[nodiscard]] Vec4d vertex(int face_idx, int vert_idx, const Painter& painter) override;
    [[nodiscard]] std::unique_ptr<Color> fragment(const Vec3d& bary_coords, const Painter& painter) const override;

  private:
    static constexpr double KA = Args::KA;
    static constexpr double KD = Args::KD;
    static constexpr double KS = Args::KS;

    std::array<Vec3d, 3> varying_view_vertex_;
    std::array<Vec3d, 3> varying_screen_vertex_;
    std::array<Vec3d, 3> varying_norm_;
    std::array<Vec2d, 3> varying_uv_;
};

class Painter
{
    using Vec2i = Eigen::Vector2i;
    using Vec2d = Eigen::Vector2d;
    using Vec3d = Eigen::Vector3d;
    using Mat4d = Eigen::Matrix4d;

  public:
    Painter(Camera&& camera, const Vec3d& light_position, double light_intensity);
    Painter(const Painter& painter)            = delete;
    Painter(Painter&& painter)                 = delete;
    Painter& operator=(const Painter& painter) = delete;
    Painter& operator=(Painter&& painter)      = delete;
    ~Painter()                                 = default;

    [[nodiscard]] const Camera& getCamera() const;
    [[nodiscard]] const Camera& getCameraAtLight() const;
    [[nodiscard]] const Mat4d& getTransformFromScreenToShadowBuffer() const;
    [[nodiscard]] const Vec3d& getLightPosition() const;
    [[nodiscard]] const Vec3d& getLightPositionInView() const;
    [[nodiscard]] const double getLightIntensity() const;
    RGBImage& getImage();
    bool viewFrustumCulling(const std::shared_ptr<Model>& model);
    bool backFaceCulling(const std::shared_ptr<Model>& model, int face_idx);
    void paint(const std::shared_ptr<Model>& model, int face_idx, Shader& shader);
    void generateShadowBuffer(ModelList& model_list);
    [[nodiscard]] bool isInShadow(const Vec3d& view_coord) const;

  private:
    Camera camera_;
    Camera camera_at_light_;
    Vec3d light_position_;
    Vec3d light_position_in_view_;
    double light_intensity_;
    std::vector<double> zbuffer_;
    std::vector<double> shadow_buffer_;
    Mat4d transform_from_view_to_shadow_buffer_;
    RGBImage image_;

    [[nodiscard]] bool validateScreenCoords(const std::array<Vec2d, 3>& screen_coords) const;
    [[nodiscard]] Vec3d barycentric(const std::array<Vec2d, 3>& screen_coords, const Vec2d& p) const;
    [[nodiscard]] std::array<Vec2i, 2> getBoundingBox(const std::array<Vec2d, 3>& screen_coords) const;
    [[nodiscard]] bool zBufferTest(int x, int y, double z) const;
    [[nodiscard]] bool shadowBufferTest(int x, int y, double z) const;
    void updateZBuffer(int x, int y, double z);
    void updateShadowBuffer(int x, int y, double z);
};
