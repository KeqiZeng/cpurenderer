#include "painter.h"
#include "camera.h"
#include "image.h"
#include "model.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

using Vec2i = Eigen::Vector2i;
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;

void Shader::setModel(std::shared_ptr<Model> model)
{
    model_.reset();
    model_ = std::move(model);
}

Vec4d PhongShader::vertex(int face_idx, int vert_idx, const Painter& painter)
{
    const auto& face   = model_->getFace(face_idx);
    const auto& vertex = model_->getVertex(face.v_idx_[vert_idx]);
    const auto& camera = painter.getCamera();
    varying_norm_[vert_idx] =
        (camera.getMVInverseTranspose() * model_->getNormal(face.n_idx_[vert_idx]).homogeneous()).hnormalized();
    varying_uv_[vert_idx] = model_->getUV(face.uv_idx_[vert_idx]);

    const Mat4d& model_view        = camera.getMVTransform();
    Vec4d view_vert4d              = model_view * vertex.homogeneous();
    varying_view_vertex_[vert_idx] = view_vert4d.hnormalized();

    return camera.getProjectionTransform() * view_vert4d;
}

std::unique_ptr<Color> PhongShader::fragment(const Vec3d& bary_coords, const Painter& painter) const
{
    // In view (camera) space
    Vec3d p = varying_view_vertex_[0] * bary_coords[0] + varying_view_vertex_[1] * bary_coords[1] +
              varying_view_vertex_[2] * bary_coords[2];
    Vec3d light_dir = (painter.getLightPositionInView() - p).normalized();
    Vec3d view_dir  = (-p).normalized();
    Vec3d half_dir  = (light_dir + view_dir).normalized();

    Vec2d uv = varying_uv_[0] * bary_coords[0] + varying_uv_[1] * bary_coords[1] + varying_uv_[2] * bary_coords[2];
    Vec3d norm =
        (varying_norm_[0] * bary_coords[0] + varying_norm_[1] * bary_coords[1] + varying_norm_[2] * bary_coords[2])
            .normalized();
    if (const auto& normal_map = model_->getNormalMap()) {
        // normal mapping, from tangent space to view space, reference:
        // https://github.com/ssloy/tinyrenderer/wiki/Lesson-6bis:-tangent-space-normal-mapping
        auto* normal = dynamic_cast<RGB*>(normal_map->getColor(uv[0], uv[1]));
        Vec3d n = Vec3d(((*normal)[0] / 255.0 * 2 - 1), ((*normal)[1] / 255.0 * 2 - 1), ((*normal)[2] / 255.0 * 2 - 1))
                      .normalized();
        Mat3d m;
        m.row(0) = (varying_view_vertex_[1] - varying_view_vertex_[0]).transpose();
        m.row(1) = (varying_view_vertex_[2] - varying_view_vertex_[0]).transpose();
        m.row(2) = norm.transpose();

        m       = m.inverse().eval();
        Vec3d u = m * Vec3d{varying_uv_[1].x() - varying_uv_[0].x(), varying_uv_[2].x() - varying_uv_[0].x(), 0};
        Vec3d v = m * Vec3d{varying_uv_[1].y() - varying_uv_[0].y(), varying_uv_[2].y() - varying_uv_[0].y(), 0};

        Mat3d transform;
        transform.col(0) = u.normalized();
        transform.col(1) = v.normalized();
        transform.col(2) = norm;

        norm = (transform * n).normalized();
        delete normal;
    }

    bool in_shadow = painter.isInShadow(p);

    auto* color   = dynamic_cast<RGB*>(model_->getDiffuseMap()->getColor(uv[0], uv[1]));
    auto* shiness = dynamic_cast<GrayScale*>(model_->getSpecularMap()->getColor(uv[0], uv[1]));

    double diffuse               = std::max(norm.dot(light_dir), 0.0);
    double specular              = std::pow(std::max(norm.dot(half_dir), 0.0), shiness->getValue());
    const double LIGHT_INTENSITY = painter.getLightIntensity();

    std::array<uint8_t, 3> rgb;
    for (int i = 0; i < 3; ++i) {
        double c = 0.0;
        if (!in_shadow) {
            c = static_cast<double>((*color)[i]) * LIGHT_INTENSITY * (KA + KD * diffuse + KS * specular);
        }
        else {
            c = static_cast<double>((*color)[i]) * LIGHT_INTENSITY * KA;
        }

        c      = std::round(std::clamp(c, 0.0, 255.0));
        rgb[i] = static_cast<uint8_t>(c);
    }

    delete color;
    delete shiness;
    return std::make_unique<RGB>(rgb[0], rgb[1], rgb[2]);
}

Painter::Painter(Camera&& camera, const Vec3d& light_position, double light_intensity)
    : camera_(std::move(camera))
    , camera_at_light_(Camera(camera_, light_position)) // camera at light, only position is different
    , light_position_(std::move(light_position))
    , light_intensity_(light_intensity)
    , zbuffer_(
          std::vector<double>(camera_.getImageWidth() * camera_.getImageHeight(), -std::numeric_limits<double>::max())
      )
    , shadow_buffer_(
          std::vector<double>(
              camera_at_light_.getImageWidth() * camera_at_light_.getImageHeight(),
              -std::numeric_limits<double>::max()
          )
      )
    , image_(camera_.getImageWidth(), camera_.getImageHeight())
{
    Mat4d model_view        = camera_.getMVTransform();
    light_position_in_view_ = (model_view * light_position_.homogeneous()).hnormalized();

    Mat4d view_to_world          = model_view.inverse();
    Mat4d world_to_shadow_buffer = camera_at_light_.getViewportTransform() * camera_at_light_.getProjectionTransform() *
                                   camera_at_light_.getMVTransform();
    transform_from_view_to_shadow_buffer_ = world_to_shadow_buffer * view_to_world;
};

const Camera& Painter::getCamera() const
{
    return camera_;
}

const Camera& Painter::getCameraAtLight() const
{
    return camera_at_light_;
}

const Vec3d& Painter::getLightPosition() const
{
    return light_position_;
}

const Vec3d& Painter::getLightPositionInView() const
{
    return light_position_in_view_;
}

const double Painter::getLightIntensity() const
{
    return light_intensity_;
}

RGBImage& Painter::getImage()
{
    return image_;
}

bool Painter::viewFrustumCulling(const std::shared_ptr<Model>& model)
{
    std::array<Vec3d, 9> vert_arr;
    vert_arr[0] = model->getBoundingBoxMin();                               // 0 0 0
    vert_arr[1] = model->getBoundingBoxMax();                               // 1 1 1
    vert_arr[2] = Vec3d(vert_arr[0].x(), vert_arr[0].y(), vert_arr[1].z()); // 0 0 1
    vert_arr[3] = Vec3d(vert_arr[0].x(), vert_arr[1].y(), vert_arr[0].z()); // 0 1 0
    vert_arr[4] = Vec3d(vert_arr[0].x(), vert_arr[1].y(), vert_arr[1].z()); // 0 1 1
    vert_arr[5] = Vec3d(vert_arr[1].x(), vert_arr[0].y(), vert_arr[0].z()); // 1 0 0
    vert_arr[6] = Vec3d(vert_arr[1].x(), vert_arr[0].y(), vert_arr[1].z()); // 1 0 1
    vert_arr[7] = Vec3d(vert_arr[1].x(), vert_arr[1].y(), vert_arr[0].z()); // 1 1 0
    vert_arr[8] = model->getBoundingBoxCenter();

    int cnt         = 0;
    const auto& mvp = camera_.getProjectionTransform() * camera_.getMVTransform();
    for (const auto& vert : vert_arr) {
        auto& vert_clip = (mvp * vert.homogeneous()).hnormalized();
        if (vert_clip.x() < -1.0 || vert_clip.x() > 1.0 || vert_clip.y() < -1.0 || vert_clip.y() > 1.0 ||
            vert_clip.z() < -1.0 || vert_clip.z() > 1.0) {
            cnt += 1;
        }
    }
    return cnt < 9;
}

bool Painter::backFaceCulling(const std::shared_ptr<Model>& model, int face_idx)
{
    const auto& face = model->getFace(face_idx);
    Vec3d a          = model->getVertex(face.v_idx_[0]);
    Vec3d b          = model->getVertex(face.v_idx_[1]);
    Vec3d c          = model->getVertex(face.v_idx_[2]);
    Vec3d norm_bary =
        (model->getNormal(face.n_idx_[0]) + model->getNormal(face.n_idx_[1]) + model->getNormal(face.n_idx_[2])) / 3.0;

    // assume vertex order is counter-clockwise
    Vec3d normal = (b - a).cross(c - a);

    if (normal.dot(norm_bary) < 0) {
        // vertex order is clockwise
        normal = -normal;
    }

    return normal.dot(camera_.getPosition() - (a + b + c) / 3) >= 0;
}

void Painter::paint(const std::shared_ptr<Model>& model, int face_idx, Shader& shader)
{
    std::array<Vec4d, 3> clip_coords4d;
    std::array<Vec2d, 3> screen_coords;
    std::array<double, 3> depth;
    for (int i = 0; i < 3; ++i) {
        clip_coords4d[i]     = shader.vertex(face_idx, i, *this);
        auto screen_coords3d = (camera_.getViewportTransform() * clip_coords4d[i]).hnormalized();
        screen_coords[i]     = Vec2d{screen_coords3d.x(), screen_coords3d.y()};
        depth[i]             = screen_coords3d.z();
    }
    if (!validateScreenCoords(screen_coords))
        // To avoid wrap around issue, simply discard the triangle if any vertex is out of screen
        return;
    auto bbox = getBoundingBox(screen_coords);
    for (int i = bbox[0][0]; i <= bbox[1][0]; ++i) {
        for (int j = bbox[0][1]; j <= bbox[1][1]; ++j) {
            auto bary_screen = barycentric(screen_coords, Vec2d{i + 0.5, j + 0.5});
            auto bary_clip   = Vec3d{
                bary_screen[0] * (1.0 / clip_coords4d[0].w()),
                bary_screen[1] * (1.0 / clip_coords4d[1].w()),
                bary_screen[2] * (1.0 / clip_coords4d[2].w()),
            };
            bary_clip /= bary_clip[0] + bary_clip[1] + bary_clip[2];
            double z = bary_clip[0] * depth[0] + bary_clip[1] * depth[1] + bary_clip[2] * depth[2];
            if (bary_screen[0] >= 0 && bary_screen[1] >= 0 && bary_screen[2] >= 0 && zBufferTest(i, j, z)) {
                updateZBuffer(i, j, z);
                auto color = shader.fragment(bary_clip, *this);
                image_.setColor(i, j, color.get());
            }
        }
    }
}

void Painter::generateShadowBuffer(ModelList& model_list)
{
    const auto& mvp = camera_at_light_.getProjectionTransform() * camera_at_light_.getMVTransform();
    for (auto& model : model_list.getModels()) {
        for (int i = 0; i < model->getFaceCount(); ++i) {
            std::array<Vec4d, 3> clip_coords4d;
            std::array<Vec2d, 3> screen_coords;
            std::array<double, 3> depth;
            for (int j = 0; j < 3; ++j) {
                const auto& vert     = model->getVertex(model->getFace(i).v_idx_[j]);
                clip_coords4d[j]     = mvp * vert.homogeneous();
                auto screen_coords3d = (camera_at_light_.getViewportTransform() * clip_coords4d[j]).hnormalized();
                screen_coords[j]     = Vec2d{screen_coords3d.x(), screen_coords3d.y()};
                depth[j]             = screen_coords3d.z();
            }
            if (!validateScreenCoords(screen_coords))
                continue;
            auto bbox = getBoundingBox(screen_coords);
            for (int i = bbox[0][0]; i <= bbox[1][0]; ++i) {
                for (int j = bbox[0][1]; j <= bbox[1][1]; ++j) {
                    auto bary_screen = barycentric(screen_coords, Vec2d{i + 0.5, j + 0.5});
                    auto bary_clip   = Vec3d{
                        bary_screen[0] * (1.0 / clip_coords4d[0].w()),
                        bary_screen[1] * (1.0 / clip_coords4d[1].w()),
                        bary_screen[2] * (1.0 / clip_coords4d[2].w()),
                    };
                    bary_clip /= bary_clip[0] + bary_clip[1] + bary_clip[2];

                    double z = bary_clip[0] * depth[0] + bary_clip[1] * depth[1] + bary_clip[2] * depth[2];
                    if (bary_screen[0] >= 0 && bary_screen[1] >= 0 && bary_screen[2] >= 0 &&
                        shadowBufferTest(i, j, z)) {
                        updateShadowBuffer(i, j, z);
                    }
                }
            }
        }
    }
}

bool Painter::isInShadow(const Vec3d& view_coord) const
{
    auto shadow_buffer_coord_with_depth =
        (transform_from_view_to_shadow_buffer_ * view_coord.homogeneous()).hnormalized();
    const double BIAS = 3.5e-3;
    int width         = image_.getWidth();
    int height        = image_.getHeight();
    if (shadow_buffer_coord_with_depth[0] < 0 || shadow_buffer_coord_with_depth[0] > width ||
        shadow_buffer_coord_with_depth[1] < 0 || shadow_buffer_coord_with_depth[1] > height) {
        return false;
    }
    int idx = static_cast<int>(shadow_buffer_coord_with_depth[0]) +
              static_cast<int>(shadow_buffer_coord_with_depth[1]) * width;
    return shadow_buffer_[idx] - BIAS > shadow_buffer_coord_with_depth[2];
}

bool Painter::validateScreenCoords(const std::array<Vec2d, 3>& screen_coords) const
{
    // check if any vertex is out of screen
    for (int i = 0; i < 3; ++i) {
        if (screen_coords[i][0] < 0 || screen_coords[i][0] > camera_.getImageWidth() || screen_coords[i][1] < 0 ||
            screen_coords[i][1] > camera_.getImageHeight())
            return false;
    }
    return true;
}

Vec3d Painter::barycentric(const std::array<Vec2d, 3>& screen_coords, const Vec2d& p) const
{
    Mat3d m;
    m << screen_coords[0].x(), screen_coords[1].x(), screen_coords[2].x(), screen_coords[0].y(), screen_coords[1].y(),
        screen_coords[2].y(), 1, 1, 1;

    const double EPSILON = 1e-3;
    if (std::abs(m.determinant()) < EPSILON)
        return Vec3d{-1, 1, 1}; // degenerate triangle

    return m.inverse() * Vec3d{p.x(), p.y(), 1};
}

std::array<Vec2i, 2> Painter::getBoundingBox(const std::array<Vec2d, 3>& screen_coords) const
{
    int width  = image_.getWidth() - 1;
    int height = image_.getHeight() - 1;
    Vec2i bbox_min{width, height};
    Vec2i bbox_max{0, 0};

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 2; ++j) {
            bbox_min[j] = std::min(bbox_min[j], static_cast<int>(screen_coords[i][j]));
            bbox_max[j] = std::max(bbox_max[j], static_cast<int>(screen_coords[i][j]));
        }
    }
    bbox_min[0] = std::clamp(bbox_min[0], 0, width);
    bbox_max[0] = std::clamp(bbox_max[0], 0, width);
    bbox_min[1] = std::clamp(bbox_min[1], 0, height);
    bbox_max[1] = std::clamp(bbox_max[1], 0, height);
    return {bbox_min, bbox_max};
}

bool Painter::zBufferTest(int x, int y, double z) const
{
    return z > zbuffer_[x + y * image_.getWidth()];
}

bool Painter::shadowBufferTest(int x, int y, double z) const
{
    return z > shadow_buffer_[x + y * image_.getWidth()];
}

void Painter::updateZBuffer(int x, int y, double z)
{
    zbuffer_[x + y * image_.getWidth()] = z;
}

void Painter::updateShadowBuffer(int x, int y, double z)
{
    shadow_buffer_[x + y * image_.getWidth()] = z;
}
