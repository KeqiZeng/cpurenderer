#include "camera.h"
#include <cmath>

using Vec3d = Eigen::Vector3d;
using Mat4d = Eigen::Matrix4d;

Camera::Camera(const Vec3d& position, const Vec3d& look_at, const Vec3d& up)
    : position_(position)
    , look_at_(look_at)
    , up_(up)
{
    lookAt();
    perspectiveProjection();
    viewport();
}

Camera::Camera(const Camera& camera, const Vec3d& position)
    : Camera(position, camera.look_at_, camera.up_)
{
}

const Vec3d& Camera::getPosition() const
{
    return position_;
}

int Camera::getImageWidth() const
{
    return width_;
}

int Camera::getImageHeight() const
{
    return height_;
}

const Mat4d& Camera::getMVTransform() const
{
    return model_view_;
}

const Mat4d& Camera::getMVInverseTranspose() const
{
    return model_view_inverse_transpose_;
}

const Mat4d& Camera::getProjectionTransform() const
{
    return projection_;
}

const Mat4d& Camera::getViewportTransform() const
{
    return viewport_;
}

void Camera::lookAt()
{
    Vec3d z = (position_ - look_at_).normalized();
    Vec3d x = up_.cross(z).normalized();
    Vec3d y = z.cross(x).normalized();
    Mat4d rotation;
    rotation << x.x(), x.y(), x.z(), 0, y.x(), y.y(), y.z(), 0, z.x(), z.y(), z.z(), 0, 0, 0, 0, 1;

    Mat4d translation = Mat4d::Identity();
    translation(0, 3) = -position_.x();
    translation(1, 3) = -position_.y();
    translation(2, 3) = -position_.z();

    model_view_                   = rotation * translation;
    model_view_inverse_transpose_ = model_view_.inverse().transpose();
}

void Camera::perspectiveProjection()
{
    double t = std::abs(n_) * std::tan(fovY_ / 2.0f * M_PI / 180.0f);
    double b = -t;

    double r = t * aspect_ratio_;
    double l = -r;

    projection_ << 2 * n_ / (r - l), 0, (l + r) / (l - r), 0, 0, 2 * n_ / (t - b), (b + t) / (b - t), 0, 0, 0,
        (f_ + n_) / (n_ - f_), 2 * f_ * n_ / (f_ - n_), 0, 0, 1, 0;
}

void Camera::viewport()
{
    int x           = width_ / 10;
    int y           = height_ / 10;
    int w           = width_ * 4 / 5;
    int h           = height_ * 4 / 5;
    viewport_       = Mat4d::Identity();
    viewport_(0, 0) = w / 2.0;
    viewport_(1, 1) = h / 2.0;
    viewport_(0, 3) = w / 2.0 + x;
    viewport_(1, 3) = h / 2.0 + y;
}
