#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <string_view>

class Color
{
  public:
    Color()                        = default;
    Color(const Color&)            = default;
    Color(Color&&)                 = default;
    Color& operator=(const Color&) = default;
    Color& operator=(Color&&)      = default;
    virtual ~Color()               = default;
    virtual void print() const     = 0;
};

class GrayScale : public Color
{
  public:
    GrayScale(uint8_t value);
    void print() const override;
    [[nodiscard]] uint8_t getValue() const;

  private:
    uint8_t value_;
};

class RGB : public Color
{
  public:
    RGB(uint8_t r, uint8_t g, uint8_t b);
    void print() const override;
    [[nodiscard]] uint8_t r() const;
    [[nodiscard]] uint8_t g() const;
    [[nodiscard]] uint8_t b() const;
    uint8_t operator[](int index) const;

  private:
    Eigen::Vector<uint8_t, 3> data_;
};

class Image
{
  public:
    Image() = default;
    Image(std::string_view filename, int channels);
    Image(int width, int height, int channels);
    Image(const Image&)            = delete;
    Image(Image&&)                 = delete;
    Image& operator=(const Image&) = delete;
    Image& operator=(Image&&)      = delete;
    virtual ~Image();

    [[nodiscard]] int getWidth() const;
    [[nodiscard]] int getHeight() const;

    [[nodiscard]] virtual Color* getColor(int x, int y) const = 0;
    [[nodiscard]] virtual Color* getColor(double x, double y) const;
    void virtual setColor(int x, int y, Color* color) = 0;
    void saveToFile(std::string_view filename, int channels);

  protected:
    int width_{0};
    int height_{0};
    uint8_t* data_{nullptr};

  private:
    void flipVertically(int channels);
};

class GrayScaleImage : public Image
{
  public:
    // getColor is defined in GrayScaleImage, every overload of getColor in Image is shadowed.
    // It is necessary to import Image::getColor, so that the getColor(double, double) of Image can be called.
    using Image::getColor;
    static constexpr int CHANNELS{1};

    GrayScaleImage(std::string_view filename);
    GrayScaleImage(int width, int height);
    GrayScaleImage(const GrayScaleImage&)            = delete;
    GrayScaleImage(GrayScaleImage&&)                 = delete;
    GrayScaleImage& operator=(const GrayScaleImage&) = delete;
    GrayScaleImage& operator=(GrayScaleImage&&)      = delete;
    ~GrayScaleImage() override                       = default;

    [[nodiscard]] Color* getColor(int x, int y) const override;
    void setColor(int x, int y, Color* color) override;
    void saveToFile(std::string_view filename);
};

class RGBImage : public Image
{
  public:
    using Image::getColor;
    static constexpr int CHANNELS{3};

    RGBImage(std::string_view filename);
    RGBImage(int width, int height);
    RGBImage(const RGBImage&)            = delete;
    RGBImage(RGBImage&&)                 = delete;
    RGBImage& operator=(const RGBImage&) = delete;
    RGBImage& operator=(RGBImage&&)      = delete;
    ~RGBImage() override                 = default;

    [[nodiscard]] Color* getColor(int x, int y) const override;
    void setColor(int x, int y, Color* color) override;
    void saveToFile(std::string_view filename);
};
