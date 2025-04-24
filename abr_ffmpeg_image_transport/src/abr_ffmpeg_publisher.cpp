// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Modified by Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

#include <ffmpeg_encoder_decoder/safe_param.hpp>
#include <abr_ffmpeg_image_transport/abr_ffmpeg_publisher.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>
#include <filesystem>
#include <fstream>

using namespace std::placeholders;

namespace abr_ffmpeg_image_transport
{
using ParameterDefinition = ABRFFMPEGPublisher::ParameterDefinition;
using ParameterValue = ABRFFMPEGPublisher::ParameterValue;
using ParameterDescriptor = ABRFFMPEGPublisher::ParameterDescriptor;

static const ParameterDefinition params[] = {
  {ParameterValue("libx264"),
    ParameterDescriptor()
    .set__name("encoding")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    .set__description("ffmpeg encoder to use, see ffmpeg h264 supported encoders")
    .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
    .set__name("preset")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    .set__description("ffmpeg encoder preset")
    .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
    .set__name("tune")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    .set__description("ffmpeg encoder tune")
    .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
    .set__name("delay")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    .set__description("ffmpeg encoder delay")
    .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
    .set__name("crf")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    .set__description("constant rate factor")
    .set__read_only(false)},
  {ParameterValue(""), ParameterDescriptor()
    .set__name("pixel_format")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    .set__description("pixel format to use for encoding")
    .set__read_only(false)},
  {ParameterValue(static_cast<int>(10)),
    ParameterDescriptor()
    .set__name("qmax")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("max video quantizer scale, see ffmpeg docs")
    .set__read_only(false)
    .set__integer_range(
      {rcl_interfaces::msg::IntegerRange().set__from_value(-1).set__to_value(1024).set__step(1)})},
  {ParameterValue(static_cast<int64_t>(8242880)),
    ParameterDescriptor()
    .set__name("bit_rate")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("target bit rate, see ffmpeg docs")
    .set__read_only(false)
    .set__integer_range({rcl_interfaces::msg::IntegerRange()
        .set__from_value(1)
        .set__to_value(std::numeric_limits<int>::max())
        .set__step(1)})},
  {ParameterValue(static_cast<int>(10)),
    ParameterDescriptor()
    .set__name("gop_size")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("gop size (distance between keyframes)")
    .set__read_only(false)
    .set__integer_range({rcl_interfaces::msg::IntegerRange()
        .set__from_value(1)
        .set__to_value(std::numeric_limits<int>::max())
        .set__step(1)})},
  {ParameterValue(false), ParameterDescriptor()
    .set__name("measure_performance")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
    .set__description("enable performance timing")
    .set__read_only(false)},
  {ParameterValue(static_cast<int>(175)),
    ParameterDescriptor()
    .set__name("performance_interval")
    .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
    .set__description("after how many frames to print perf info")
    .set__read_only(false)
    .set__integer_range({rcl_interfaces::msg::IntegerRange()
        .set__from_value(1)
        .set__to_value(std::numeric_limits<int>::max())
        .set__step(1)})},
};

ABRFFMPEGPublisher::ABRFFMPEGPublisher()
: logger_(rclcpp::get_logger("ABRFFMPEGPublisher"))
{
  framerate_ts = rclcpp::Time(0, 0, RCL_STEADY_TIME);
}

ABRFFMPEGPublisher::~ABRFFMPEGPublisher() {}

// This code was lifted from compressed_image_transport

void ABRFFMPEGPublisher::declareParameter(
  rclcpp::Node * node, const std::string & base_name, const ParameterDefinition & definition)
{
  // transport scoped parameter (e.g. image_raw.compressed.format)
  const std::string transport_name = getTransportName();
  const std::string param_name =
    base_name + "." + transport_name + "." + definition.descriptor.name;
  rclcpp::ParameterValue v;
  try {
    v = node->declare_parameter(param_name, definition.defaultValue, definition.descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG(logger_, "%s was previously declared", definition.descriptor.name.c_str());
    v = node->get_parameter(param_name).get_parameter_value();
  }
  const auto & n = definition.descriptor.name;
  if (n == "encoding") {
    encoder_.setEncoder(v.get<std::string>());
    RCLCPP_INFO_STREAM(logger_, "using encoder: " << v.get<std::string>());
  } else if (n == "preset") {
    encoder_.setPreset(v.get<std::string>());
  } else if (n == "tune") {
    encoder_.setTune(v.get<std::string>());
  } else if (n == "delay") {
    encoder_.setDelay(v.get<std::string>());
  } else if (n == "crf") {
    encoder_.setCRF(v.get<std::string>());
  } else if (n == "pixel_format") {
    encoder_.setPixelFormat(v.get<std::string>());
  } else if (n == "qmax") {
    encoder_.setQMax(v.get<int>());
  } else if (n == "bit_rate") {
    encoder_.setBitRate(v.get<int>());
  } else if (n == "gop_size") {
    encoder_.setGOPSize(v.get<int>());
  } else if (n == "measure_performance") {
    measurePerformance_ = v.get<bool>();
    encoder_.setMeasurePerformance(v.get<bool>());
  } else if (n == "performance_interval") {
    performanceInterval_ = v.get<int>();
  } else {
    RCLCPP_ERROR_STREAM(logger_, "unknown parameter: " << n);
  }
}

void ABRFFMPEGPublisher::packetReady(
  const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
  uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz)
{
  auto msg = std::make_shared<FFMPEGPacket>();
  msg->header.frame_id = frame_id;
  msg->header.stamp = stamp;
  msg->encoding = codec;
  msg->width = width;
  msg->height = height;
  msg->pts = pts;
  msg->flags = flags;
  msg->data.assign(data, data + sz);

  (*publishFunction_)(*msg);
}

#if defined(IMAGE_TRANSPORT_API_V1) || defined(IMAGE_TRANSPORT_API_V2)
void ABRFFMPEGPublisher::advertiseImpl(
  rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos)
{
  auto qos = initialize(node, base_topic, custom_qos);
  FFMPEGPublisherPlugin::advertiseImpl(node, base_topic, qos);
}
#else
void ABRFFMPEGPublisher::advertiseImpl(
  rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions opt)
{
  auto qos = initialize(node, base_topic, custom_qos);
  FFMPEGPublisherPlugin::advertiseImpl(node, base_topic, qos, opt);
}
#endif

rmw_qos_profile_t ABRFFMPEGPublisher::initialize(
  rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos)
{

  /*
  Load json posible resolutions resolutions
  */
  std::filesystem::path source_dir = std::filesystem::path(__FILE__).parent_path().parent_path();
  std::filesystem::path file_path = source_dir / "json/resolution_configurations.json";
  std::ifstream file1(file_path);

  if (!file1.is_open()) {
    throw std::runtime_error("Could not open the resolution configuration file.");
  }
  nlohmann::json json_config;
  file1 >> json_config;

  for (const auto & [key, value] : json_config.items()) {
    int width = value["width"];
    int height = value["height"];
    resolution_map_[key] = {width, height};
  }


  /*
  Load json bitladder
  */

  file_path = source_dir / "json/bitrate_ladder.json";
  std::ifstream file2(file_path);
  if (!file2.is_open()) {
    std::cerr << "Couldnt open json file" << std::endl;
  }
  nlohmann::json configurations;
  file2 >> configurations;
  file2.close();

  for (const auto & value : configurations) {
    bitrate_ladder.push_back(value.get<double>());
  }


  /*
  Initialize ABR communication and configure QoS for info channel
  */

  rclcpp::QoS qos_settings(10);  // Depth de 10
  qos_settings.reliability(rclcpp::ReliabilityPolicy::Reliable);  // Reliable
  qos_settings.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Transient Local

  abr_info_publisher_ = node->create_publisher<abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket>("abr_info", qos_settings);
  abr_info_subscriber_ = node->create_subscription<abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket>(
    "abr_info", qos_settings,
    [this, node](const abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket::SharedPtr msg) {
      this->abrInfoCallback(msg);
    }
  );
  /*
  Change CODEC parameters:
  */

  const std::string ns = "abr_ffmpeg_image_transport.";
  measurePerformance_ = ffmpeg_encoder_decoder::get_safe_param<bool>(node,
      ns + "measure_performance", false);
  encoder_.setMeasurePerformance(measurePerformance_);
  performanceInterval_ = ffmpeg_encoder_decoder::get_safe_param<int>(node,
      ns + "performance_interval", 175);


  /*
  Change VIDEO QoS parameters:
  */
  int depth = 0;
  rclcpp::QoS custom_qos_profile = rclcpp::QoS(rclcpp::KeepLast(depth));
  custom_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  custom_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

  custom_qos = custom_qos_profile.get_rmw_qos_profile();

  node_ = node;
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  auto current_time = steady_clock.now();

  init_ready = true;

  // namespace handling code lifted from compressed_image_transport
  const uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

  for (const auto & p : params) {
    declareParameter(node, param_base_name, p);
  }
  // bump queue size to 2 * distance between keyframes
  custom_qos.depth = std::max(static_cast<int>(custom_qos.depth), 2 * encoder_.getGOPSize());
  return  custom_qos;
}


std::string ABRFFMPEGPublisher::identifyResolution(int width, int height) const
{
  /**
 * @brief Identifies the closest resolution name based on the given width and height.
 *
 * This function first checks for an exact match of the specified `width` and `height`
 * within `resolution_map_`. If no exact match is found, it then calculates the aspect
 * ratio of the provided dimensions and finds the closest matching resolution based on
 * the smallest difference in aspect ratio.
 *
 * @param width The width of the resolution to identify.
 * @param height The height of the resolution to identify.
 * @return A `std::string` representing the name of the closest matching resolution.
 */

  double best_diff = std::numeric_limits<double>::max();
  std::string best_res_name = "Unknown";

  for (const auto & [res_name, dims] : resolution_map_) {
    if (width == dims.first && height == dims.second) {
      return res_name;
    }
  }

  for (const auto & [res_name, dims] : resolution_map_) {
    double aspect_ratio_diff = std::abs(static_cast<double>(width) / height -
        static_cast<double>(dims.first) / dims.second);
    if (aspect_ratio_diff < best_diff) {
      best_diff = aspect_ratio_diff;
      best_res_name = res_name;
    }
  }

  RCLCPP_INFO(logger_, "No exact match found, using closest resolution: %s (%d x %d)",
              best_res_name.c_str(), resolution_map_.at(best_res_name).first,
      resolution_map_.at(best_res_name).second);
  return best_res_name;
}


std::pair<int, int> ABRFFMPEGPublisher::parseResolution(const std::string & res_str) const
{
    /**
     * @brief Parses a resolution string to retrieve its corresponding width and height.
     *
     * This function looks up a resolution string (e.g., "1080p") in the `resolution_map_`
     * and returns the corresponding resolution dimensions as a pair of integers. If the
     * resolution string is not found, an error is logged, and a runtime exception is thrown.
     *
     * @param res_str A string representing the desired resolution (e.g., "720p", "1080p").
     * @return A `std::pair<int, int>` containing the width and height corresponding to the resolution.
     * @throws std::runtime_error if the resolution string is not found in `resolution_map_`.
     */

  auto it = resolution_map_.find(res_str);
  if (it != resolution_map_.end()) {
    return it->second;
  } else {
    RCLCPP_ERROR(logger_, "@parseResolution: Unknown resolution string: %s", res_str.c_str());
    throw std::runtime_error("Unknown resolution string: " + res_str);
  }
}


void ABRFFMPEGPublisher::expectedBitrateLadder(int width, int height, int framerate) const
{

  /**
   * @brief Calculates and updates the expected bitrate ladder based on resolution and framerate.
   *
   * This function recalculates the bitrate ladder values using a specified resolution (`width` x `height`)
   * and `framerate`, combined with predefined compression ratios. The result is stored in an updated
   * `bitrate_ladder` vector, which is then used for adaptive bitrate control.
   *
   * @param width The width of the video resolution.
   * @param height The height of the video resolution.
   * @param framerate The target framerate of the video.
   */

  std::vector<double> updated_bitrate_ladder;
  updated_bitrate_ladder.reserve(bitrate_ladder.size());

  for (const auto & compression_ratio : bitrate_ladder) {

    double expected_bitrate = static_cast<double>(width) * height * framerate * compression_ratio;
    updated_bitrate_ladder.push_back(expected_bitrate);
    RCLCPP_INFO(logger_, "Calculated expected bitrate: %f with compression factor: %f",
        expected_bitrate, compression_ratio);
  }

  bitrate_ladder = std::move(updated_bitrate_ladder);
}


void ABRFFMPEGPublisher::filterBitrateLadder(double max_rate_mbps) const
{
  bitrate_ladder.erase(
        std::remove_if(bitrate_ladder.begin(), bitrate_ladder.end(),
    [max_rate_mbps](double bitrate) {return bitrate > max_rate_mbps;}),
        bitrate_ladder.end()
  );

  RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "Filtered bitrate ladder:");
  for (const auto & bitrate : bitrate_ladder) {
    RCLCPP_INFO(rclcpp::get_logger("AbrComponent"), "%f Mbps", bitrate);
  }
}


void ABRFFMPEGPublisher::publish(const Image & msg, const PublishFn & publish_fn) const
{
  ABRFFMPEGPublisher * me = const_cast<ABRFFMPEGPublisher *>(this);
  me->publishFunction_ = &publish_fn;

  // Check if the system is ready to receive video
  if (ready_to_recive_video) {

    if (!me->encoder_.isInitialized()) {
      if (!me->encoder_.initialize(
            msg.width, msg.height,
            std::bind(&ABRFFMPEGPublisher::packetReady, me, _1, _2, _3, _4, _5, _6, _7, _8, _9)))
      {
        RCLCPP_ERROR_STREAM(logger_, "cannot initialize encoder!");
        return;
      }
    }
    // may trigger packetReady() callback(s) from encoder!
    me->encoder_.encodeImage(msg);

    if (measurePerformance_) {
      if (static_cast<int>(++me->frameCounter_) > performanceInterval_) {
        me->encoder_.printTimers(logger_.get_name());
        me->encoder_.resetTimers();
        me->frameCounter_ = 0;
      }
    }
  } else {
    if (init_ready) {

      // Load configuration settings from 'json'
      std::filesystem::path source_dir =
        std::filesystem::path(__FILE__).parent_path().parent_path();
      std::filesystem::path file_path = source_dir / "json/config.json";
      std::ifstream file1(file_path);
      if (!file1.is_open()) {
        throw std::runtime_error("Could not open the configuration file.");
      }
      nlohmann::json json_config;
      file1 >> json_config;
      file1.close();

      // Set framerate based on configuration file or calculate dynamically if not defined
      if (json_config.contains("target_framerate")) {
        if (json_config["target_framerate"].is_number()) {

          framerate = json_config["target_framerate"].get<int>();
          RCLCPP_INFO(logger_, "Using target framerate from config.json: %f fps", framerate);
        } else if (json_config["target_framerate"].is_string() &&
          json_config["target_framerate"] == "")
        {

          // Calculate framerate dynamically
          RCLCPP_INFO(logger_, "Target framerate is empty, estimating dynamically.");
          static std::vector<int> framerate_values;
          rclcpp::Clock steady_clock(RCL_STEADY_TIME);
          auto current_time = steady_clock.now();

          double elapsed_time = (current_time - last_update_time).seconds();
          if (elapsed_time >= 1.0) {
            if (!framerate_values.empty()) {
              double harmonic_sum = 0.0;
              for (int fr : framerate_values) {
                harmonic_sum += 1.0 / static_cast<double>(fr);
              }

              double harmonic_mean_framerate = framerate_values.size() / harmonic_sum;
              RCLCPP_INFO(logger_, "Harmonic mean framerate over 1 second: %.2f fps",
                  harmonic_mean_framerate);

              framerate = static_cast<int>(std::floor(harmonic_mean_framerate));
            }

            framerate_values.clear();
            last_update_time = current_time;
          }

          double time_diff = (current_time - framerate_ts).seconds();
          if (time_diff > 0.001) {
            int new_framerate = std::round(1.0 / time_diff);

            if (new_framerate > 1) {
              framerate_values.push_back(new_framerate);
              RCLCPP_INFO(logger_, "Calculated framerate: %d fps", new_framerate);
            } else {
              RCLCPP_WARN(logger_, "Invalid framerate: %d fps (too low)", new_framerate);
            }
          }

          framerate_ts = current_time;
        } else {
          throw std::runtime_error("Invalid type for 'target_framerate' in config.json");
        }
      } else {
        throw std::runtime_error("Missing 'target_framerate' in config.json");
      }


      // Set encoding parameters if the framerate is valid
      if (framerate >= 1.0) {

        if (framerate > 120) {
          framerate = 120; // Limit framerate to 120 fps
        }

        // Load encoder settings from the config file
        width = msg.width;
        height = msg.height;
        std::string resolution = identifyResolution(msg.width, msg.height);

        std::filesystem::path source_dir =
          std::filesystem::path(__FILE__).parent_path().parent_path();

        std::filesystem::path file_path = source_dir / "json/config.json";
        std::ifstream file1(file_path);
        if (!file1.is_open()) {
          throw std::runtime_error("Could not open the configuration file.");
        }
        nlohmann::json json_config;
        file1 >> json_config;


        double max_rate_mbps = json_config.value("max_rate_mbps", 0.0);
        if (max_rate_mbps == 0.0 ||
          (json_config["max_rate_mbps"].is_string() &&
          json_config["max_rate_mbps"].get<std::string>().empty()))
        {
          max_rate_mbps = std::numeric_limits<double>::max();   // Set to a very high value if max_rate_mbps is 0 or empty string
        }
        filterBitrateLadder(max_rate_mbps);

        nlohmann::json json_bitrate_ladder;
        for (const auto & bitrate : bitrate_ladder) {
          json_bitrate_ladder.push_back(bitrate);
        }

        bitrate_ladder_string = json_bitrate_ladder.dump();
        ladder_ready = true;


        // Find the configuration with the lowest bitrate
        if (!bitrate_ladder.empty()) {
          selected_bitrate = *std::min_element(bitrate_ladder.begin(), bitrate_ladder.end()) * 1e6;
          RCLCPP_INFO(logger_, "Selected bitrate (smallest in list): %d bps", selected_bitrate);
        } else {
          RCLCPP_ERROR(logger_, "bitrate_ladder is empty, cannot set selected_bitrate.");
        }

        //Load from config.json

        // node_->set_parameter(rclcpp::Parameter("encoding", json_config.value("codecName", "libx264")));
        // node_->set_parameter(rclcpp::Parameter("preset", json_config.value("codec_preset", "fast")));
        // node_->set_parameter(rclcpp::Parameter("tune", "zerolatency"));
        // node_->set_parameter(rclcpp::Parameter("gop_size", json_config.value("gop_size", 3)));
        // node_->set_parameter(rclcpp::Parameter("frame_rate", static_cast<double>(framerate)));
        // node_->set_parameter(rclcpp::Parameter("bit_rate", selected_bitrate));


        me->encoder_.setPreset(json_config.value("codecName", "libx264"));
        me->encoder_.setPreset(json_config.value("codec_preset", "fast"));
        me->encoder_.setTune("zerolatency");
        me->encoder_.setBitRate(selected_bitrate);
        me->encoder_.setGOPSize(json_config.value("gop_size", 3));
        me->encoder_.setFrameRate(static_cast<double>(framerate), 1);


        file1.close();

        forced_width = width;
        forced_height = height;

        RCLCPP_INFO(logger_, "Best resolution match found, forced_width: %d, forced_height: %d",
            forced_width, forced_height);

        ready_to_recive_video = true;
      }
    }

  }
}


/*
  _____ _   _ ______ ____      _____          _      _      ____          _____ _  __
 |_   _| \ | |  ____/ __ \    / ____|   /\   | |    | |    |  _ \   /\   / ____| |/ /
   | | |  \| | |__ | |  | |  | |       /  \  | |    | |    | |_) | /  \ | |    | ' /
   | | | . ` |  __|| |  | |  | |      / /\ \ | |    | |    |  _ < / /\ \| |    |  <
  _| |_| |\  | |   | |__| |  | |____ / ____ \| |____| |____| |_) / ____ \ |____| . \
 |_____|_| \_|_|    \____/    \_____/_/    \_\______|______|____/_/    \_\_____|_|\_\

*/

void ABRFFMPEGPublisher::abrInfoCallback(
  const abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket::SharedPtr msg)
{
  /**
 * @brief Callback function for handling incoming ABRInfoPacket messages related to Adaptive Bitrate (ABR) requests.
 *
 * This callback function processes ABR requests received from the client. It handles different message types:
 * - Type 0 (HANDSHAKE): Responds with information about the available bitrate ladder, target width, height, and framerate.
 * - Type 1 (QUALITY_CHANGE): Adjusts the encoding quality based on the desired bitrate specified by the client.
 *
 * @param msg The incoming ABRInfoPacket message pointer.
 */

  if (msg->role == "client") {

    if (msg->msg_type == 0) {  // HANDSHAKE

      nlohmann::json json_msg = nlohmann::json::parse(msg->msg_json);

      if (ladder_ready) {  // Check if JSON is empty or matches your expected structure
        abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket init_msg;
        init_msg.role = "server";
        init_msg.msg_type = 0;

        nlohmann::json response_json = {
          {"available", true},
          {"bitrate_ladder", bitrate_ladder_string},
          {"desired_width", width},
          {"desired_height", height},
          {"framerate", framerate}

        };

        init_msg.msg_json = response_json.dump();
        abr_info_publisher_->publish(init_msg);

      } else {

        abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket init_msg;
        init_msg.role = "server";
        init_msg.msg_type = 0;

        nlohmann::json response_json = {
          {"available", false},
          {"bitrate_ladder", ""}
        };

        init_msg.msg_json = response_json.dump();
        abr_info_publisher_->publish(init_msg);
      }


    } else if (msg->msg_type == 1) { // QUALITY_CHANGE: Adjust codec parameters to match requested bitrate

      RCLCPP_INFO(logger_, "Changing the quality");

      nlohmann::json json_msg = nlohmann::json::parse(msg->msg_json);

      if (!json_msg.empty()) {

        double desired_bitrate = json_msg["desired_bitrate"].get<double>();
        auto it = std::lower_bound(bitrate_ladder.begin(), bitrate_ladder.end(), desired_bitrate);

        if (it == bitrate_ladder.end()) {
          RCLCPP_WARN(logger_, "Desired bitrate not found in ladder, unable to change quality.");
          return;
        }

        double selected_bitrate = *it;
        RCLCPP_INFO(logger_, "New configuration ASKED: Bitrate: %f", selected_bitrate);

        abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket init_msg;
        init_msg.role = "server";
        init_msg.msg_type = 1;

        nlohmann::json response_json = {
          {"confirmed", true},
          {"selected_bitrate", selected_bitrate}
        };
        init_msg.msg_json = response_json.dump();

          //////////////////////////////////////////////////
          // encoder_.closeCodec();
        ABRFFMPEGPublisher * me = const_cast<ABRFFMPEGPublisher *>(this);
        me->encoder_.reset();
        me->encoder_.setBitRate(selected_bitrate * 1e6);

          // if (!encoder_.openCodec(forced_width, forced_height)) {
          //     RCLCPP_ERROR_STREAM(logger_, "Failed to reopen codec with new parameters!");
          // }
        if (!me->encoder_.initialize(
            forced_width, forced_height,
            std::bind(&ABRFFMPEGPublisher::packetReady, me, _1, _2, _3, _4, _5, _6, _7, _8, _9)))
        {
          RCLCPP_ERROR_STREAM(logger_, "Failed to reopen codec with new parameters!");
        }
          //////////////////////////////

        abr_info_publisher_->publish(init_msg);
      }
    }
  }
}

}  // namespace abr_ffmpeg_image_transport
