#pragma once

#include <GL/glew.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <tuple>

namespace lidar_viewer
{
struct PanoramaCanvas
{
  int x, y, width, height;
};

struct PanoramaLayerKey
{
  PanoramaCanvas canvas;
  GLuint vao;
  std::uint64_t upload_revision;
  std::size_t first_point, point_count;
  int color_mode;
  bool light_theme;
  std::array<float, 4> view, appearance;
  std::array<float, 9> sensor_rotation;
  std::array<float, 3> sensor_translation;

  auto Values() const
  {
    return std::tie(canvas.x, canvas.y, canvas.width, canvas.height, vao, upload_revision,
      first_point, point_count, color_mode, light_theme, view, appearance, sensor_rotation,
      sensor_translation);
  }
};

class PanoramaRenderCache
{
public:
  enum class Paint { Draw, Capture, Reuse };

  PanoramaRenderCache()
  {
    const auto * renderer = reinterpret_cast<const char *>(glGetString(GL_RENDERER));
    const std::string_view name = renderer ? renderer : "";
    // Measured dense panoramas benefit on software GL and Mesa Intel. Blitting
    // can cost more than drawing a zoomed cloud directly on a faster GPU.
    if (name.find("llvmpipe") == name.npos && name.find("softpipe") == name.npos &&
      name.find("Mesa Intel") == name.npos)
    {
      return;
    }
    GLint sample_buffers = 0;
    glGetIntegerv(GL_SAMPLE_BUFFERS, &sample_buffers);
    if (sample_buffers || glIsEnabled(GL_FRAMEBUFFER_SRGB)) return;
    for (const GLenum component : {GL_FRAMEBUFFER_ATTACHMENT_RED_SIZE,
           GL_FRAMEBUFFER_ATTACHMENT_GREEN_SIZE, GL_FRAMEBUFFER_ATTACHMENT_BLUE_SIZE,
           GL_FRAMEBUFFER_ATTACHMENT_ALPHA_SIZE})
    {
      GLint bits = 0;
      glGetFramebufferAttachmentParameteriv(GL_FRAMEBUFFER, GL_BACK_LEFT, component, &bits);
      if (bits > 8) return;
    }
    glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &max_extent_);
  }

  PanoramaRenderCache(const PanoramaRenderCache &) = delete;
  PanoramaRenderCache & operator=(const PanoramaRenderCache &) = delete;
  ~PanoramaRenderCache() {Release();}

  bool CanReuse(PanoramaCanvas rect, int framebuffer_width, int framebuffer_height,
    std::size_t points) const
  {
    // For light point clouds the cost of copying the entire canvas can dominate.
    return rect.width <= max_extent_ && rect.height <= max_extent_ &&
      rect.x >= 0 && rect.y >= 0 && rect.x + rect.width <= framebuffer_width &&
      rect.y + rect.height <= framebuffer_height &&
      points >= (static_cast<std::size_t>(rect.width) * rect.height) / 16;
  }

  Paint Prepare(const PanoramaLayerKey & layer)
  {
    if (!layer_ || layer_->Values() != layer.Values()) {
      layer_ = layer;
      captured_ = false;
      return Paint::Draw;
    }
    // Capture only after the draw key repeats. A pan, zoom, or rapid seek can
    // keep drawing directly without allocating or copying on each change.
    if (captured_) return Paint::Reuse;
    return EnsureStorage(layer.canvas.width, layer.canvas.height) ? Paint::Capture : Paint::Draw;
  }

  void Invalidate()
  {
    layer_.reset();
    captured_ = false;
  }

  void Capture(PanoramaCanvas rect)
  {
    // The UI has not been drawn yet and scissoring is disabled. Save the exact
    // back-buffer pixels, including the driver-specific point rasterization.
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer_);
    glBlitFramebuffer(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height,
      0, 0, rect.width, rect.height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    captured_ = true;
  }

  void Present(PanoramaCanvas rect) const
  {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer_);
    glBlitFramebuffer(0, 0, rect.width, rect.height,
      rect.x, rect.y, rect.x + rect.width, rect.y + rect.height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  }

  void Release()
  {
    Invalidate();
    if (framebuffer_) glDeleteFramebuffers(1, &framebuffer_);
    if (color_buffer_) glDeleteRenderbuffers(1, &color_buffer_);
    framebuffer_ = color_buffer_ = 0;
    width_ = height_ = 0;
  }

private:
  bool EnsureStorage(int width, int height)
  {
    if (width == width_ && height == height_) return true;
    if (!framebuffer_) glGenFramebuffers(1, &framebuffer_);
    if (!color_buffer_) glGenRenderbuffers(1, &color_buffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, color_buffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width, height);
    GLint allocated_width = 0, allocated_height = 0;
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &allocated_width);
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &allocated_height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer_);
    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_buffer_);
    const bool complete = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    if (!complete || allocated_width != width || allocated_height != height) {
      Release();
      max_extent_ = 0;
      return false;
    }
    width_ = width;
    height_ = height;
    return true;
  }

  GLuint framebuffer_{0}, color_buffer_{0};
  int width_{0}, height_{0}, max_extent_{0};
  std::optional<PanoramaLayerKey> layer_;
  bool captured_{false};
};
}  // namespace lidar_viewer
