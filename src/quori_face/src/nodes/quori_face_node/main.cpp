#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <atomic>
#include <chrono>

#include <unistd.h>

#include "quori_face/Window.hpp"
#include "quori_face/WindowManager.hpp"
#include "quori_face/Shader.hpp"
#include "quori_face/Program.hpp"
#include "quori_face/Texture.hpp"
#include "quori_face/PixelBufferObject.hpp"
#include "quori_face/transform.hpp"
#include "quori_face/gl.hpp"

#include "geometry.hpp"
#include "shader/frag.hpp"
#include "shader/vertex.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <unordered_map>
#include <ros/ros.h>
#include <ros/package.h>

#include <boost/optional.hpp>
#include <fstream>

#include <dynamic_reconfigure/server.h>
#include <quori_face/CalibrationConfig.h>

#include <yaml-cpp/yaml.h>

#include "trace.hpp"
#include "param.hpp"

using namespace quori_face;
using namespace quori_face_node;

namespace
{
  sensor_msgs::ImageConstPtr latest_image;

  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    latest_image = msg;
  }

  std::atomic<bool> static_params_updated(false);
  TransformStaticParameters static_params(TransformStaticParameters::DEFAULT);

  SphericalCoordinate center(SphericalCoordinate::CENTER);

  SphericalCoordinate min_coord = {
    .theta = -M_PI * 0.45,
    .psi = -M_PI * 0.5
  };

  SphericalCoordinate max_coord = {
    .theta = M_PI * 0.45,
    .psi = M_PI * 0.5
  };

  void reconfigureCallback(CalibrationConfig &calibration, uint32_t level)
  {
    static bool debounce = true;
    if (debounce)
    {
      debounce = false;
      return;
    }

    static_params.delta.x = calibration.dx;
    static_params.delta.y = calibration.dy;
    
    center.theta = calibration.center_theta;
    center.psi = calibration.center_psi;
    
    min_coord.theta = center.theta + calibration.min_theta;
    min_coord.psi = center.psi + calibration.min_psi;

    max_coord.theta = center.theta + calibration.max_theta;
    max_coord.psi = center.psi + calibration.max_psi;

    static_params_updated = true;
  }

  const Vector2<std::uint32_t> LOOKUP_TABLE_RESOLUTION = {
    // Theta resolution
    .x = 2048,
    // Psi resolution
    .y = 2048
  };

  const Vector2<std::uint32_t> IMAGE_RESOLUTION = {
    .x = 1280,
    .y = 720
  };

  const std::unordered_map<std::string, std::uint32_t> ENCODING_GL_MAPPINGS {
    { sensor_msgs::image_encodings::RGB8, GL_RGB },
    { sensor_msgs::image_encodings::BGR8, GL_BGR },
  };

  const std::unordered_map<std::string, std::uint32_t> ENCODING_CV_MAPPINGS {
    { sensor_msgs::image_encodings::RGB8, CV_8UC3 },
    { sensor_msgs::image_encodings::BGR8, CV_8UC3 }
  };

  const std::unordered_map<std::string, boost::optional<cv::ColorConversionCodes>> ENCODING_CV_CVT_MAPPINGS {
    { sensor_msgs::image_encodings::RGB8, cv::COLOR_RGB2BGR },
    { sensor_msgs::image_encodings::BGR8, boost::none }
  };

  std::size_t encodingSize(const std::string &encoding)
  {
    const std::size_t bitDepth = sensor_msgs::image_encodings::bitDepth(latest_image->encoding);
    const std::size_t numChannels = sensor_msgs::image_encodings::numChannels(latest_image->encoding);
    return bitDepth / 8 * numChannels;
  }

  void mapImage(std::uint8_t *const data, const cv::Mat &image)
  {
    const std::size_t elementSize = image.elemSize();
    for (std::size_t row = 0; row < image.rows; ++row)
    {
      memcpy(data + image.cols * row * elementSize, image.ptr(row), image.cols * elementSize);
    }
  }
}



int main(int argc, char *argv[])
{
  using namespace std;
  using namespace cv;

  ros::init(argc, argv, "quori_face");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  

  boost::recursive_mutex reconfigure_mutex;
  dynamic_reconfigure::Server<quori_face::CalibrationConfig> reconfigure_server(reconfigure_mutex);
  reconfigure_server.setCallback(boost::bind(&reconfigureCallback, _1, _2));

  const auto lookup_table_resolution = param(pnh, "lookup_table_resolution", LOOKUP_TABLE_RESOLUTION);
  const auto image_resolution = param(pnh, "image_resolution", IMAGE_RESOLUTION);
  center = param(pnh, "center", center);

  min_coord = center + param(pnh, "min", min_coord);
  max_coord = center + param(pnh, "max", max_coord);

  const bool update_params = param(pnh, "update_params", false);

  const std::string override_image_encoding = param(pnh, "override_image_encoding", std::string());

  static_params = param(pnh, "transform", static_params);



  CalibrationConfig config;
  config.dx = static_params.delta.x;
  config.dy = static_params.delta.y;
  config.center_theta = center.theta;
  config.center_psi = center.psi;
  config.min_theta = center.theta - min_coord.theta;
  config.min_psi = center.psi - min_coord.psi;
  config.max_theta = center.theta - max_coord.theta;
  config.max_psi = center.psi - max_coord.psi;
  reconfigure_server.updateConfig(config);

  const auto generate_lookup_table = [&]() {
    const auto start_time = std::chrono::system_clock::now();
    cout << "Updating lookup table (this may take several seconds)...";
    cout.flush();
    
    GLfloat *const ret = generateLookupTable(static_params, min_coord, max_coord, lookup_table_resolution);
    const auto end_time = std::chrono::system_clock::now();
    cout << " done (took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << "ms)" << endl;
    return ret;
  };
  GLfloat *lookup_table = generate_lookup_table();


  image_transport::ImageTransport it(pnh);
  const auto image_sub = it.subscribe("image", 1, &imageCallback);
  const auto window = Window::open(static_params.screen_size.x, static_params.screen_size.y, "Quori Face", Monitor::getPrimaryMonitor());
  QUORI_FACE_NODE_TRACE(glEnable(GL_TEXTURE_2D));

  window->bind();

  WindowManager::ref().attach(window);
  const auto program = Program::link({
    Shader::compile(Shader::Type::Vertex, shader::VERTEX),
    Shader::compile(Shader::Type::Fragment, shader::FRAGMENT)
  });

  // Generate geometry for displaying the transformed image

  GLuint vao;
  QUORI_FACE_NODE_TRACE(glGenVertexArrays(1, &vao));

  GLuint vbo;
  QUORI_FACE_NODE_TRACE(glGenBuffers(1, &vbo));

  GLuint ebo;
  QUORI_FACE_NODE_TRACE(glGenBuffers(1, &ebo));

  QUORI_FACE_NODE_TRACE(glBindVertexArray(vao));

  QUORI_FACE_NODE_TRACE(glBindBuffer(GL_ARRAY_BUFFER, vbo));
  QUORI_FACE_NODE_TRACE(glBufferData(GL_ARRAY_BUFFER, GEOMETRY_VERTICES.size() * sizeof(float), GEOMETRY_VERTICES.data(), GL_STATIC_DRAW));

  QUORI_FACE_NODE_TRACE(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo));
  QUORI_FACE_NODE_TRACE(glBufferData(GL_ELEMENT_ARRAY_BUFFER, GEOMETRY_INDICES.size() * sizeof(std::uint32_t), GEOMETRY_INDICES.data(), GL_STATIC_DRAW));

  QUORI_FACE_NODE_TRACE(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), nullptr));
  QUORI_FACE_NODE_TRACE(glEnableVertexAttribArray(0));

  QUORI_FACE_NODE_TRACE(glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), (void *)(3 * sizeof(GLfloat))));
  QUORI_FACE_NODE_TRACE(glEnableVertexAttribArray(1));

  // Create textures

  size_t image_size = image_resolution.x * image_resolution.y * 3;
  uint8_t *const image_texture_data = new uint8_t[image_size];
  memset(image_texture_data, 0, image_size);
  Texture::Ptr image_texture = Texture::create(image_resolution.y, image_resolution.x, GL_BGR, image_texture_data);
  delete[] image_texture_data;
  const auto lookup_table_texture = Texture::create(static_params.screen_size.y, static_params.screen_size.x, lookup_table);

  program->use();
  QUORI_FACE_NODE_TRACE(glUniform1i(program->getUniformLocation("lookup_table"), 0));
  QUORI_FACE_NODE_TRACE(glUniform1i(program->getUniformLocation("image"), 1));
  GLint i_resolution[] = {
    static_cast<GLint>(static_params.screen_size.x),
    static_cast<GLint>(static_params.screen_size.y),
    0
  };

  QUORI_FACE_NODE_TRACE(glUniform3iv(program->getUniformLocation("i_resolution"), 3, i_resolution));
  QUORI_FACE_NODE_TRACE(glActiveTexture(GL_TEXTURE0));
  QUORI_FACE_NODE_TRACE(glBindBuffer(GL_ARRAY_BUFFER, 0)); 
  QUORI_FACE_NODE_TRACE(glBindVertexArray(0));

  std::size_t image_pbo_index = 0;
  std::size_t lookup_table_pbo_index = 0;
  PixelBufferObject image_pbo(image_resolution.x * image_resolution.y * sizeof(uint8_t) * 3);
  PixelBufferObject lookup_table_pbo(lookup_table_resolution.x * lookup_table_resolution.y * sizeof(float) * 3);

  const auto copy_current_image_pbo = [&]()
  {
    image_texture->bind();
    image_pbo.bind(image_pbo_index);
    QUORI_FACE_NODE_TRACE(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image_resolution.x, image_resolution.y, GL_BGR, GL_UNSIGNED_BYTE, 0));
    checkGlError();
    image_pbo.unbind();
  };

  const auto write_next_image_pbo = [&](const cv::Mat &image)
  {
    image_pbo.bind(image_pbo_index + 1);
    uint8_t *const data = reinterpret_cast<uint8_t *>(QUORI_FACE_NODE_TRACE(glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_READ_WRITE)));
    checkGlError();
    if (data)
    {
      mapImage(data, image);
      QUORI_FACE_NODE_TRACE(glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER));
      checkGlError();
    }

    image_pbo.unbind();
  };

  sensor_msgs::ImageConstPtr prev_image;

  while (!window->isClosed() && ros::ok())
  {
    ros::spinOnce();

    if (!latest_image) continue;
    if (latest_image == prev_image) continue;

    const auto &encoding = override_image_encoding.empty() ? latest_image->encoding : override_image_encoding;

    const auto cv_it = ENCODING_CV_MAPPINGS.find(encoding);
    if (cv_it == ENCODING_CV_MAPPINGS.cend())
    {
      cerr << "Unsupported image encoding (no CV encoding) \"" << encoding << "\"" << endl;
      continue;
    }


    const cv::Mat image(
      cv::Size(latest_image->width, latest_image->height),
      cv_it->second,
      // We pinky promise not to modify the constant image data
      const_cast<uint8_t *>(latest_image->data.data()),
      latest_image->step
    );
    
    const auto cvt_it = ENCODING_CV_CVT_MAPPINGS.find(encoding);
    if (cvt_it == ENCODING_CV_CVT_MAPPINGS.cend())
    {
      cerr << "Unsupported image encoding (no conversion) \"" << encoding << "\"" << endl;
      continue;
    }

    cv::Mat final_image;
    if (!cvt_it->second)
    {
      final_image = image;
    }
    else
    {
      cv::cvtColor(image, final_image, cvt_it->second.get());
    }

    if (final_image.rows != image_resolution.y || final_image.cols != image_resolution.x)
    {
      cv::Mat resized;
      cv::resize(final_image, resized, cv::Size(image_resolution.x, image_resolution.y));
      final_image = resized;
    }

    prev_image = latest_image;

    if (static_params_updated)
    {
      static_params_updated = false;
      GLfloat *const next_lookup_table = generate_lookup_table();

      QUORI_FACE_NODE_TRACE(glActiveTexture(GL_TEXTURE0));
      checkGlError();

      lookup_table_texture->bind();
      QUORI_FACE_NODE_TRACE(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, static_params.screen_size.x, static_params.screen_size.y, GL_RGB, GL_FLOAT, next_lookup_table));
      delete[] lookup_table;
      lookup_table = next_lookup_table;
      checkGlError();

      glBindTexture(GL_TEXTURE_2D, 0);
      checkGlError();
    }


    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    QUORI_FACE_NODE_TRACE(glActiveTexture(GL_TEXTURE1));

    copy_current_image_pbo();
    write_next_image_pbo(final_image);

    program->use();

    QUORI_FACE_NODE_TRACE(glActiveTexture(GL_TEXTURE0));
    lookup_table_texture->bind();

    QUORI_FACE_NODE_TRACE(glActiveTexture(GL_TEXTURE1));
    image_texture->bind();
    
    QUORI_FACE_NODE_TRACE(glBindVertexArray(vao));
    QUORI_FACE_NODE_TRACE(glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0));
    QUORI_FACE_NODE_TRACE(glBindVertexArray(0));

    window->swapBuffers();
    ++image_pbo_index;


    Window::pollEvents();
  }

  QUORI_FACE_NODE_TRACE(glDeleteVertexArrays(1, &vao));
  QUORI_FACE_NODE_TRACE(glDeleteBuffers(1, &vbo));
  QUORI_FACE_NODE_TRACE(glDeleteBuffers(1, &ebo));

  delete[] lookup_table;

  // Update params

  if (update_params)
  {
    const std::string path = ros::package::getPath("quori_face");
    if (path.empty())
    {
      ROS_ERROR("Failed to update parameter file. quori_face package not found.");
      return EXIT_FAILURE;
    }

    std::ofstream out(path + "/config/params.yaml");

    if (!out.is_open())
    {
      ROS_ERROR("Failed to update parameter file. Couldn't open parameter file for writing.");
      return EXIT_FAILURE;
    }

    YAML::Node delta;
    delta["x"] = static_params.delta.x;
    delta["y"] = static_params.delta.y;

    YAML::Node screen_size;
    screen_size["x"] = static_params.screen_size.x;
    screen_size["y"] = static_params.screen_size.y;
    
    YAML::Node transform;
    transform["R"] = static_params.R;
    transform["r_m"] = static_params.r_m;
    transform["r_o"] = static_params.r_o;
    transform["h"] = static_params.h;
    transform["L"] = static_params.L;
    transform["epsilon"] = static_params.epsilon;
    transform["delta"] = delta;
    transform["screen_size"] = screen_size;

    YAML::Node lookup_table_resolution_node;
    lookup_table_resolution_node["x"] = lookup_table_resolution.x;
    lookup_table_resolution_node["y"] = lookup_table_resolution.y;

    YAML::Node image_resolution_node;
    image_resolution_node["x"] = image_resolution.x;
    image_resolution_node["y"] = image_resolution.y;

    YAML::Node min_node;
    min_node["theta"] = (min_coord.theta - center.theta);
    min_node["psi"] = (min_coord.psi - center.psi);

    YAML::Node max_node;
    max_node["theta"] = (max_coord.theta - center.theta);
    max_node["psi"] = (max_coord.psi - center.psi);

    YAML::Node center_node;
    center_node["theta"] = center.theta;
    center_node["psi"] = center.psi;

    YAML::Node root;
    root["transform"] = transform;
    root["lookup_table_resolution"] = lookup_table_resolution_node;
    root["image_resolution"] = image_resolution_node;
    root["max"] = max_node;
    root["min"] = min_node;
    root["center"] = center_node;


    YAML::Emitter emitter;
    emitter << root;

    out << emitter.c_str();
    out.close();
  }

  return EXIT_SUCCESS;
}