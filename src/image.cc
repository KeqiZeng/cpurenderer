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

int Image::getWidth() const
{
    return width_;
}

int Image::getHeight() const
{
    return height_;
}

static Color* lerp(Color* a, Color* b, double t)
{
    auto* gray_scale_a = dynamic_cast<GrayScale*>(a);
    auto* gray_scale_b = dynamic_cast<GrayScale*>(b);
    auto* rgb_a        = dynamic_cast<RGB*>(a);
    auto* rgb_b        = dynamic_cast<RGB*>(b);
    if (gray_scale_a && gray_scale_b) {
        auto value = static_cast<uint8_t>(gray_scale_a->getValue() * (1 - t) + gray_scale_b->getValue() * t);
        return new GrayScale(value);
    }
    if (rgb_a && rgb_b) {
        auto r = static_cast<uint8_t>(rgb_a->r() * (1 - t) + rgb_b->r() * t);
        auto g = static_cast<uint8_t>(rgb_a->g() * (1 - t) + rgb_b->g() * t);
        auto b = static_cast<uint8_t>(rgb_a->b() * (1 - t) + rgb_b->b() * t);
        return new RGB(r, g, b);
    }
    throw std::runtime_error("Invalid color type");
}

Color* Image::getColor(double x, double y) const
{
    // x, y in [0, 1]
    x *= width_;                                   // [0, width]
    y *= height_;                                  // [0, height]
    int x_round = static_cast<int>(std::round(x)); // [0, width]
    int y_round = static_cast<int>(std::round(y)); // [0, height]
    double dx   = 1.0 - (x_round + 0.5 - x);
    double dy   = 1.0 - (y_round + 0.5 - y);

    int x0 = std::clamp(x_round - 1, 0, width_ - 1);
    int x1 = std::clamp(x_round, 0, width_ - 1);
    int y0 = std::clamp(y_round - 1, 0, height_ - 1);
    int y1 = std::clamp(y_round, 0, height_ - 1);

    // "this" pointer is used to call the member function of the derived class
    auto* color00 = this->getColor(x0, y0);
    auto* color01 = this->getColor(x0, y1);
    auto* color10 = this->getColor(x1, y0);
    auto* color11 = this->getColor(x1, y1);

    Color* color0 = nullptr;
    Color* color1 = nullptr;
    Color* color  = nullptr;
    try {
        color0 = lerp(color00, color10, dx);
        color1 = lerp(color01, color11, dx);
        color  = lerp(color0, color1, dy);
    }
    catch (std::runtime_error& e) {
        delete color00;
        delete color01;
        delete color10;
        delete color11;
        delete color0;
        delete color1;
        throw e;
    }

    delete color00;
    delete color01;
    delete color10;
    delete color11;
    delete color0;
    delete color1;

    return color;
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

Color* GrayScaleImage::getColor(int x, int y) const
{
    assert(x >= 0 && x < width_ && y >= 0 && y < height_);
    return new GrayScale(data_[(x + y * width_) * CHANNELS]);
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

Color* RGBImage::getColor(int x, int y) const
{
    assert(x >= 0 && x < width_ && y >= 0 && y < height_);
    int idx   = (x + y * width_) * CHANNELS;
    uint8_t r = data_[idx];
    uint8_t g = data_[idx + 1];
    uint8_t b = data_[idx + 2];
    return new RGB(r, g, b);
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
