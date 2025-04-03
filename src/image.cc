#include "image.h"
#include "stb_image.h"
#include "stb_image_write.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string_view>

GrayScale::GrayScale(uint8_t value)
    : value_(value)
{
}

void GrayScale::print() const
{
    std::cout << "GrayScale: " << static_cast<int>(value_) << std::endl;
}

uint8_t GrayScale::getValue() const
{
    return value_;
}

RGB::RGB(uint8_t r, uint8_t g, uint8_t b)
    : data_(Eigen::Vector<uint8_t, 3>(r, g, b))
{
}

void RGB::print() const
{
    std::cout << "RGB: (" << static_cast<int>(data_[0]) << " " << static_cast<int>(data_[1]) << " "
              << static_cast<int>(data_[2]) << ")" << std::endl;
}

uint8_t RGB::r() const
{
    return data_[0];
}

uint8_t RGB::g() const
{
    return data_[1];
}

uint8_t RGB::b() const
{
    return data_[2];
}

uint8_t RGB::operator[](int index) const
{
    return data_[index];
}

Image::Image(std::string_view filename, int channels)
    : data_(stbi_load(filename.data(), &width_, &height_, nullptr, channels))
{
    if (!data_) {
        throw std::runtime_error("Failed to load image, error: " + std::string(stbi_failure_reason()));
    }
    flipVertically(channels);
}

Image::Image(int width, int height, int channels)
    : width_(width)
    , height_(height)
    , data_(new uint8_t[width * height * channels])
{
}

Image::~Image()
{
    stbi_image_free(data_);
}

[[nodiscard]] int Image::getWidth() const
{
    return width_;
}

[[nodiscard]] int Image::getHeight() const
{
    return height_;
}

void Image::saveToFile(std::string_view filename, int channels)
{
    // check ext
    if (filename.substr(filename.find_last_of(".") + 1) != "png") {
        throw std::runtime_error("Invalid file extension");
    }
    flipVertically(channels);
    stbi_write_png(filename.data(), width_, height_, channels, data_, width_ * channels);
}

void Image::flipVertically(int channels)
{
    int bytes_per_row = width_ * channels;
    auto* temp_row    = (uint8_t*)malloc(bytes_per_row);

    for (int y = 0; y < height_ / 2; ++y) {
        uint8_t* top_row    = data_ + y * bytes_per_row;
        uint8_t* bottom_row = data_ + (height_ - 1 - y) * bytes_per_row;

        memcpy(temp_row, top_row, bytes_per_row);
        memcpy(top_row, bottom_row, bytes_per_row);
        memcpy(bottom_row, temp_row, bytes_per_row);
    }

    free(temp_row);
}

GrayScaleImage::GrayScaleImage(std::string_view filename)
    : Image(filename, CHANNELS)
{
}

GrayScaleImage::GrayScaleImage(int width, int height)
    : Image(width, height, CHANNELS)
{
}

[[nodiscard]] Color* GrayScaleImage::getColor(int x, int y) const
{
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return nullptr;
    }
    return new GrayScale(data_[(x + y * width_) * CHANNELS]);
}

[[nodiscard]] Color* GrayScaleImage::getColor(double x, double y) const
{
    // x, y in [0, 1]
    x *= width_;                                     // [0, width]
    y *= height_;                                    // [0, height]
    int x_round   = static_cast<int>(std::round(x)); // [0, width]
    int y_round   = static_cast<int>(std::round(y)); // [0, height]
    double dx     = x_round + 0.5 - x;
    double dy     = y_round + 0.5 - y;
    auto* color00 = dynamic_cast<GrayScale*>(getColor(x_round - 1, y_round - 1));
    auto* color01 = dynamic_cast<GrayScale*>(getColor(x_round - 1, y_round));
    auto* color10 = dynamic_cast<GrayScale*>(getColor(x_round, y_round - 1));
    auto* color11 = dynamic_cast<GrayScale*>(getColor(x_round, y_round));

    uint8_t value = 0;
    // 8 special case to handle: 4 corners and 4 edges and 1 normal case
    if (color00 == nullptr && color01 == nullptr && color10 == nullptr && color11 != nullptr) {
        // bottom left corner
        value = color11->getValue();
        delete color11;
    }
    else if (color00 == nullptr && color01 == nullptr && color10 != nullptr && color11 != nullptr) {
        // left edge
        value = static_cast<uint8_t>(color10->getValue() * dy + color11->getValue() * (1 - dy));
        delete color10;
        delete color11;
    }
    else if (color00 == nullptr && color01 == nullptr && color10 != nullptr && color11 == nullptr) {
        // top left corner
        value = color10->getValue();
        delete color10;
    }
    else if (color00 != nullptr && color01 == nullptr && color10 != nullptr && color11 == nullptr) {
        // top edge
        value = static_cast<uint8_t>(color00->getValue() * dx + color10->getValue() * (1 - dx));
        delete color00;
        delete color10;
    }
    else if (color00 != nullptr && color01 == nullptr && color10 == nullptr && color11 == nullptr) {
        // top right corner
        value = color00->getValue();
        delete color00;
    }
    else if (color00 != nullptr && color01 != nullptr && color10 == nullptr && color11 == nullptr) {
        // right edge
        value = static_cast<uint8_t>(color00->getValue() * dy + color01->getValue() * (1 - dy));
        delete color10;
        delete color11;
    }
    else if (color00 == nullptr && color01 != nullptr && color10 == nullptr && color11 == nullptr) {
        // bottom right corner
        value = color01->getValue();
        delete color01;
    }
    else if (color00 == nullptr && color01 != nullptr && color10 == nullptr && color11 != nullptr) {
        // bottom edge
        value = static_cast<uint8_t>(color01->getValue() * dx + color11->getValue() * (1 - dx));
        delete color01;
        delete color11;
    }
    else if (color00 != nullptr && color01 != nullptr && color10 != nullptr && color11 != nullptr) {
        value = static_cast<uint8_t>(
            color00->getValue() * dx * dy + color01->getValue() * dx * (1 - dy) + color10->getValue() * (1 - dx) * dy +
            color11->getValue() * (1 - dx) * (1 - dy)
        );
        delete color00;
        delete color01;
        delete color10;
        delete color11;
    }
    return new GrayScale(value);
}

void GrayScaleImage::setColor(int x, int y, Color* color)
{
    if (auto* gray_scale = dynamic_cast<GrayScale*>(color)) {
        data_[(x + y * width_) * CHANNELS] = gray_scale->getValue();
    }
    else {
        throw std::runtime_error("Invalid color type");
    }
}

void GrayScaleImage::saveToFile(std::string_view filename)
{
    Image::saveToFile(filename, CHANNELS);
}

RGBImage::RGBImage(std::string_view filename)
    : Image(filename, CHANNELS)
{
}

RGBImage::RGBImage(int width, int height)
    : Image(width, height, CHANNELS)
{
}

[[nodiscard]] Color* RGBImage::getColor(int x, int y) const
{
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return nullptr;
    }
    int idx   = (x + y * width_) * CHANNELS;
    uint8_t r = data_[idx];
    uint8_t g = data_[idx + 1];
    uint8_t b = data_[idx + 2];
    return new RGB(r, g, b);
}

[[nodiscard]] Color* RGBImage::getColor(double x, double y) const
{
    x *= width_;                                     // [0, width]
    y *= height_;                                    // [0, height]
    int x_round   = static_cast<int>(std::round(x)); // [0, width]
    int y_round   = static_cast<int>(std::round(y)); // [0, height]
    double dx     = x_round + 0.5 - x;
    double dy     = y_round + 0.5 - y;
    auto* color00 = dynamic_cast<RGB*>(getColor(x_round - 1, y_round - 1));
    auto* color01 = dynamic_cast<RGB*>(getColor(x_round - 1, y_round));
    auto* color10 = dynamic_cast<RGB*>(getColor(x_round, y_round - 1));
    auto* color11 = dynamic_cast<RGB*>(getColor(x_round, y_round));

    std::array<uint8_t, 3> rgb;
    // 8 special case to handle: 4 corners and 4 edges and 1 normal case
    if (color00 == nullptr && color01 == nullptr && color10 == nullptr && color11 != nullptr) {
        // bottom left corner
        for (int i = 0; i < 3; ++i) {
            rgb[i] = (*color11)[i];
        }
        delete color11;
    }
    else if (color00 == nullptr && color01 == nullptr && color10 != nullptr && color11 != nullptr) {
        // left edge
        for (int i = 0; i < 3; ++i) {
            rgb[i] = static_cast<uint8_t>((*color10)[i] * dy + (*color11)[i] * (1 - dy));
        }
        delete color10;
        delete color11;
    }
    else if (color00 == nullptr && color01 == nullptr && color10 != nullptr && color11 == nullptr) {
        // top left corner
        for (int i = 0; i < 3; ++i) {
            rgb[i] = (*color10)[i];
        }
        delete color10;
    }
    else if (color00 != nullptr && color01 == nullptr && color10 != nullptr && color11 == nullptr) {
        // top edge
        for (int i = 0; i < 3; ++i) {
            rgb[i] = static_cast<uint8_t>((*color00)[i] * dx + (*color10)[i] * (1 - dx));
        }
        delete color00;
        delete color10;
    }
    else if (color00 != nullptr && color01 == nullptr && color10 == nullptr && color11 == nullptr) {
        // top right corner
        for (int i = 0; i < 3; ++i) {
            rgb[i] = (*color00)[i];
        }
        delete color00;
    }
    else if (color00 != nullptr && color01 != nullptr && color10 == nullptr && color11 == nullptr) {
        // right edge
        for (int i = 0; i < 3; ++i) {
            rgb[i] = static_cast<uint8_t>((*color00)[i] * dy + (*color01)[i] * (1 - dy));
        }
        delete color00;
        delete color01;
    }
    else if (color00 == nullptr && color01 != nullptr && color10 == nullptr && color11 == nullptr) {
        // bottom right corner
        for (int i = 0; i < 3; ++i) {
            rgb[i] = (*color01)[i];
        }
        delete color01;
    }
    else if (color00 == nullptr && color01 != nullptr && color10 == nullptr && color11 != nullptr) {
        // bottom edge
        for (int i = 0; i < 3; ++i) {
            rgb[i] = static_cast<uint8_t>((*color01)[i] * dx + (*color11)[i] * (1 - dx));
        }
        delete color01;
        delete color11;
    }
    else if (color00 != nullptr && color01 != nullptr && color10 != nullptr && color11 != nullptr) {
        for (int i = 0; i < 3; ++i) {
            rgb[i] = static_cast<uint8_t>(
                (*color00)[i] * dx * dy + (*color01)[i] * dx * (1 - dy) + (*color10)[i] * (1 - dx) * dy +
                (*color11)[i] * (1 - dx) * (1 - dy)
            );
        }
        delete color00;
        delete color01;
        delete color10;
        delete color11;
    }
    return new RGB(rgb[0], rgb[1], rgb[2]);
}

void RGBImage::setColor(int x, int y, Color* color)
{
    if (auto* rgb = dynamic_cast<RGB*>(color)) {
        int idx        = (x + y * width_) * CHANNELS;
        data_[idx]     = rgb->r();
        data_[idx + 1] = rgb->g();
        data_[idx + 2] = rgb->b();
    }
    else if (auto* gray_scale = dynamic_cast<GrayScale*>(color)) {
        int idx        = (x + y * width_) * CHANNELS;
        uint8_t value  = gray_scale->getValue();
        data_[idx]     = value;
        data_[idx + 1] = value;
        data_[idx + 2] = value;
    }
    else {
        throw std::runtime_error("Invalid color type");
    }
}

void RGBImage::saveToFile(std::string_view filename)
{
    Image::saveToFile(filename, CHANNELS);
}
