#pragma once

#include "args.h"
#include <Eigen/Dense>

class Camera
{
    using Vec3d = Eigen::Vector3d;
    using Mat4d = Eigen::Matrix4d;

  public:
    Camera(const Vec3d& position, const Vec3d& look_at, const Vec3d& up);
    Camera(const Camera& camera, const Vec3d& position);

    [[nodiscard]] const Vec3d& getPosition() const;
    [[nodiscard]] int getImageWidth() const;
    [[nodiscard]] int getImageHeight() const;
    [[nodiscard]] const Mat4d& getMVTransform() const;
    [[nodiscard]] const Mat4d& getMVInverseTranspose() const;
    [[nodiscard]] const Mat4d& getProjectionTransform() const;
    [[nodiscard]] const Mat4d& getViewportTransform() const;

  private:
    Vec3d position_;
    Vec3d look_at_;
    Vec3d up_;

    double fovY_{Args::FOV_Y};
    double aspect_ratio_{Args::ASPECT_RATIO};
    double n_{Args::NEAR};
    double f_{Args::FAR};

    int width_{Args::IMG_WIDTH};
    int height_{Args::IMG_HEIGHT};

    Mat4d model_view_;
    Mat4d model_view_inverse_transpose_;
    Mat4d projection_;
    Mat4d viewport_;

    void lookAt();
    void perspectiveProjection();
    void viewport();
};
