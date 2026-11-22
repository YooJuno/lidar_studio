#include "gl_programs.hpp"
#include "panorama_palette.hpp"

#include <array>
#include <stdexcept>
#include <string>

namespace lidar_viewer::rendering
{
namespace
{
GLuint CompileShader(GLenum type, const char * source)
{
  const GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);

  GLint ok = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (ok == GL_TRUE) return shader;

  GLint log_length = 0;
  glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
  std::string log(static_cast<std::size_t>(log_length), '\0');
  glGetShaderInfoLog(shader, log_length, nullptr, log.data());
  glDeleteShader(shader);
  throw std::runtime_error("Shader compile failed:\n" + log);
}

GLuint LinkProgram(const char * vertex_source, const char * fragment_source, const char * name)
{
  const GLuint vertex = CompileShader(GL_VERTEX_SHADER, vertex_source);
  GLuint fragment;
  try {
    fragment = CompileShader(GL_FRAGMENT_SHADER, fragment_source);
  } catch (...) {
    glDeleteShader(vertex);
    throw;
  }

  const GLuint program = glCreateProgram();
  glAttachShader(program, vertex);
  glAttachShader(program, fragment);
  glLinkProgram(program);
  glDetachShader(program, vertex);
  glDetachShader(program, fragment);
  glDeleteShader(vertex);
  glDeleteShader(fragment);

  GLint ok = GL_FALSE;
  glGetProgramiv(program, GL_LINK_STATUS, &ok);
  if (ok == GL_TRUE) return program;

  GLint log_length = 0;
  glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_length);
  std::string log(static_cast<std::size_t>(log_length), '\0');
  glGetProgramInfoLog(program, log_length, nullptr, log.data());
  glDeleteProgram(program);
  throw std::runtime_error(std::string(name) + " program link failed:\n" + log);
}
}  // namespace

GLuint CreatePointProgram()
{
  constexpr const char * kVertexShader = R"(
    #version 330 core
    layout(location = 0) in vec3 a_position;
    layout(location = 1) in float a_intensity;

    uniform mat4 u_mvp;
    uniform float u_point_size;
    uniform int u_first_ground;
    uniform bool u_panorama;
    uniform mat3 u_sensor_rotation;
    uniform vec3 u_sensor_translation;
    uniform vec4 u_panorama_view; // bearing/elevation center, horizontal/vertical FOV in degrees
    out float v_intensity;
    out float v_height;
    out float v_range;
    layout(location = 2) in int a_label;
    flat out int v_label;
    flat out int v_ground;

    void main() {
      gl_PointSize = u_point_size;
      if(u_panorama) {
        vec3 sensor=transpose(u_sensor_rotation)*(a_position-u_sensor_translation);
        v_range=length(sensor);
        if(isnan(v_range) || isinf(v_range) || v_range<=0.05) {
          gl_Position=vec4(2,2,2,1);
          return;
        }
        // Same spherical geometry as SemanticKITTI's range projection. The
        // depth buffer keeps the nearest return; no sort or CPU image copy.
        float bearing=sensor.x==0.0 ? -90.0*sign(sensor.y) : -degrees(atan(sensor.y,sensor.x));
        float delta=mod(bearing-u_panorama_view.x+180.0,360.0)-180.0;
        float horizontal=length(sensor.xy);
        float elevation=horizontal==0.0 ? 90.0*sign(sensor.z) : degrees(atan(sensor.z,horizontal));
        gl_Position=vec4(2.0*delta/u_panorama_view.z,
          2.0*(elevation-u_panorama_view.y)/u_panorama_view.w,
          2.0*v_range/(v_range+1.0)-1.0,1.0);
      } else {
        gl_Position = u_mvp * vec4(a_position, 1.0);
        v_range=0.0;
      }
      v_intensity = a_intensity;
      v_height = a_position.z;
      v_label = a_label;
      v_ground = int(u_first_ground >= 0 && gl_VertexID >= u_first_ground);
    }
  )";

  constexpr const char * kFragmentShader = R"(
    #version 330 core
    in float v_intensity;
    in float v_height;
    in float v_range;
    flat in int v_label;
    flat in int v_ground;
    out vec4 frag_color;

    uniform int u_color_mode;
    uniform float u_min_height;
    uniform float u_max_height;
    uniform float u_range_max;
    uniform vec3 u_range_colors[16];
    uniform vec4 u_label_colors[8];
    uniform vec4 u_unlabeled_color;
    uniform vec4 u_background_color;
    uniform vec4 u_plain_color;

    vec3 turbo(float x) {
      x = clamp(x, 0.0, 1.0);
      return vec3(
        0.13572138 + 4.61539260 * x - 42.66032258 * x * x + 132.13108234 * x * x * x - 152.94239396 * x * x * x * x + 59.28637943 * x * x * x * x * x,
        0.09140261 + 2.19418839 * x + 4.84296658 * x * x - 14.18503333 * x * x * x + 4.27729857 * x * x * x * x + 2.82956604 * x * x * x * x * x,
        0.10667330 + 12.64194608 * x - 60.58204836 * x * x + 110.36276771 * x * x * x - 89.90310912 * x * x * x * x + 27.34824973 * x * x * x * x * x
      );
    }

    void main() {
      if (u_color_mode == 3) {
        frag_color=v_label<0 ? u_unlabeled_color : v_label==0 ? u_background_color : u_label_colors[(v_label-1)%8];
        return;
      }
      if (v_ground != 0) {
        frag_color = vec4(1.0, 0.12, 0.08, 1.0);
        return;
      }
      if (u_color_mode == 4) {
        const int last = u_range_colors.length() - 1;
        float segment = clamp(v_range / max(u_range_max, 0.1), 0.0, 1.0) * float(last);
        int left = min(int(segment), last - 1);
        frag_color = vec4(mix(u_range_colors[left], u_range_colors[left + 1], segment - float(left)), 1.0);
        return;
      }
      if (u_color_mode == 2) {
        frag_color = u_plain_color;
        return;
      }
      float value = v_intensity;
      if (u_color_mode == 1) {
        float denom = max(1e-6, u_max_height - u_min_height);
        value = (v_height - u_min_height) / denom;
      }
      frag_color = vec4(clamp(turbo(value), 0.0, 1.0), 1.0);
    }
  )";

  const GLuint program = LinkProgram(kVertexShader, kFragmentShader, "Point");
  glUseProgram(program);
  constexpr int count = IM_ARRAYSIZE(kPanoramaRangeColors);
  static_assert(count == 16, "Match u_range_colors in the point shader");
  std::array<float, count * 3> range_colors{};
  for (int i = 0; i < count; ++i) {
    const auto color = ImGui::ColorConvertU32ToFloat4(kPanoramaRangeColors[i]);
    range_colors[i * 3] = color.x;
    range_colors[i * 3 + 1] = color.y;
    range_colors[i * 3 + 2] = color.z;
  }
  glUniform3fv(glGetUniformLocation(program, "u_range_colors[0]"), count, range_colors.data());
  return program;
}

GLuint CreateGridProgram()
{
  constexpr const char * kVertexShader = R"(
    #version 330 core
    layout(location = 0) in vec3 a_position;
    uniform mat4 u_mvp;
    void main() {
      gl_Position = u_mvp * vec4(a_position, 1.0);
    }
  )";

  constexpr const char * kFragmentShader = R"(
    #version 330 core
    uniform vec4 u_color;
    out vec4 frag_color;
    void main() {
      frag_color = u_color;
    }
  )";

  return LinkProgram(kVertexShader, kFragmentShader, "Grid");
}
}  // namespace lidar_viewer::rendering
