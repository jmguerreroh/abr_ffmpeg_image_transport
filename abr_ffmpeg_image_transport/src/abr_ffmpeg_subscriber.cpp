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
#include <abr_ffmpeg_image_transport/abr_ffmpeg_subscriber.hpp>
#include <functional>
#include <unordered_map>

using namespace std::placeholders;

namespace abr_ffmpeg_image_transport
{
ABRFFMPEGSubscriber::ABRFFMPEGSubscriber()
: logger_(rclcpp::get_logger("ABRFFMPEGSubscriber")) {}

ABRFFMPEGSubscriber::~ABRFFMPEGSubscriber()
{
  abr_component_.reset();
  decoder_.reset(); // Reset decoder to adapt to new configurations
  decoder_.resetTimers();
}

void ABRFFMPEGSubscriber::frameReady(const ImageConstPtr & img, bool) const {(*userCallback_)(img);}

#ifdef IMAGE_TRANSPORT_API_V1
void ABRFFMPEGSubscriber::subscribeImpl(
  rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
  rmw_qos_profile_t custom_qos)
{
  initialize(node, base_topic);
  FFMPEGSubscriberPlugin::subscribeImpl(node, base_topic, callback, custom_qos);
}
#else
void ABRFFMPEGSubscriber::subscribeImpl(
  rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
  rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions opt)
{
  initialize(node, base_topic);
#ifdef IMAGE_TRANSPORT_API_V2
  (void)opt;  // to suppress compiler warning
  FFMPEGSubscriberPlugin::subscribeImpl(node, base_topic, callback, custom_qos);
#else
  FFMPEGSubscriberPlugin::subscribeImpl(node, base_topic, callback, custom_qos, opt);
#endif
}
#endif

void ABRFFMPEGSubscriber::initialize(rclcpp::Node * node, const std::string & base_topic)
{
  node_ = node;
  // Declare Parameters
  uint ns_len = node->get_effective_namespace().length();
  std::string param_base_name = base_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  param_namespace_ = param_base_name + "." + getTransportName() + ".map.";
  // create parameters from default map
  for (const auto & kv : ffmpeg_encoder_decoder::Decoder::getDefaultEncoderToDecoderMap()) {
    const std::string key = param_namespace_ + kv.first;
    rclcpp::ParameterValue v;
    try {
      rcl_interfaces::msg::ParameterDescriptor pd;
      pd.set__name(kv.first)
      .set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
      .set__description("ffmpeg decoder map entry")
      .set__read_only(false);
      v = node->declare_parameter(key, rclcpp::ParameterValue(kv.second), pd);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      RCLCPP_DEBUG_STREAM(logger_, "was previously declared: " << kv.first);
      v = node->get_parameter(key).get_parameter_value();
    }
    if (v.get<std::string>() != kv.second) {
      RCLCPP_INFO_STREAM(
        logger_, "overriding default decoder " << kv.second << " for " << kv.first << " with "
                                               << v.get<std::string>());
    }
  }
  const bool mp = ffmpeg_encoder_decoder::get_safe_param<bool>(
    node_, param_namespace_ + "measure_performance", false);
  decoder_.setMeasurePerformance(mp);


  /*
  Info channel config and QoS settings
  */

  allow_transmition_ = false;
  rclcpp::QoS qos_settings(10);
  qos_settings.reliability(rclcpp::ReliabilityPolicy::Reliable);  // Reliable
  qos_settings.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Transient Local

  abr_info_publisher_ = node->create_publisher<abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket>("abr_info", qos_settings);
  abr_info_subscriber_ = node->create_subscription<abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket>(
      "abr_info", qos_settings,
      std::bind(&ABRFFMPEGSubscriber::abrInfoCallback, this, std::placeholders::_1));

  // Send an initial "init" message
  abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket init_msg;
  init_msg.role = "client";
  init_msg.msg_type = 0;
  init_msg.msg_json = "{}";
  abr_info_publisher_->publish(init_msg);

  abr_component_.publish_msg_ =
    [this](const abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket & msg) {
      this->abr_info_publisher_->publish(msg);
    };

}

void ABRFFMPEGSubscriber::internalCallback(
  const FFMPEGPacketConstPtr & msg,
  const Callback & user_cb)
{
  if (allow_transmition_) {
    if (!decoder_.isInitialized()) {
      if (msg->flags == 0) {
        RCLCPP_ERROR_STREAM(logger_, "waiting for key frame");
        abr_component_.updateUnestabilityBuffer(true);
        abr_component_.analyzeFlow(*msg, node_);
        return;  // wait for key frame!
      }
      if (msg->encoding.empty()) {
        RCLCPP_ERROR_STREAM(logger_, "no encoding provided!");
        return;
      }
      userCallback_ = &user_cb;
      const std::string decoder = ffmpeg_encoder_decoder::get_safe_param<std::string>(
        node_, param_namespace_ + msg->encoding, "");
      if (decoder.empty()) {
        RCLCPP_ERROR_STREAM(logger_, "no valid decoder found for encoding: " << msg->encoding);
        return;
      }
      if (!decoder_.initialize(
            msg->encoding, std::bind(&ABRFFMPEGSubscriber::frameReady, this, _1, _2), decoder))
      {
        RCLCPP_ERROR_STREAM(logger_, "cannot initialize decoder!");
        return;
      }
    }

    abr_component_.analyzeFlow(*msg, node_);
    decoder_.decodePacket(
      msg->encoding, &msg->data[0], msg->data.size(), msg->pts, msg->header.frame_id,
      msg->header.stamp);
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


void ABRFFMPEGSubscriber::abrInfoCallback(
  const abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket::SharedPtr msg)
{

  /**
 * @brief Handles Adaptive Bitrate (ABR) messages from the server, manages handshake, and processes configuration updates.
 *
 * This function is responsible for handling different types of ABR messages, such as
 * handshake and new configuration requests. It parses JSON data from the message,
 * updates settings in the ABR component, configures bitrate ladder, and manages
 * codec adjustments based on new settings received from the server.
 *
 * @param msg The incoming ABRInfoPacket message containing the role, type, and configuration data.
 */


  if (msg->role == "server") {

    if (msg->msg_type == 0) {  // HANDSHAKE

      //Parses json
      nlohmann::json json_msg = nlohmann::json::parse(msg->msg_json);
      if (json_msg["available"].get<bool>()) {

        //Positive confirmation of the publisher
        RCLCPP_INFO(logger_, "Handshake finished successfully");

        //Obtain the bitrate ladder and map it.
        std::string bitrate_ladder_string = json_msg["bitrate_ladder"];
        nlohmann::json bitrate_ladder_json = nlohmann::json::parse(bitrate_ladder_string);

        bitrate_ladder.clear();
        for (const auto & value : bitrate_ladder_json) {
          bitrate_ladder.push_back(value.get<double>());
        }

        desired_width = json_msg["desired_width"];
        desired_height = json_msg["desired_height"];

        framerate = json_msg.value("framerate", abr_component_.getfps());
        abr_component_.setfps(framerate);

        RCLCPP_INFO(logger_, "Desired width: %d, Desired height: %d and desired framerate: %d",
            desired_width, desired_height, framerate);

        /*
        Load params from config file
        */
        std::filesystem::path source_dir =
          std::filesystem::path(__FILE__).parent_path().parent_path();
        std::filesystem::path file_path = source_dir / "json/config.json";

        std::ifstream file1(file_path);
        if (!file1.is_open()) {
          throw std::runtime_error("Could not open the resolution configuration file.");
        }

        nlohmann::json json_config;
        file1 >> json_config;
        abr_component_.setKFactor(json_config.value("abr_k_factor", abr_component_.getKFactor()));

        abr_component_.setBitrateTimeWindow(json_config.value("abr_bitrate_time_window",
            abr_component_.getBitrateTimeWindow()));
        abr_component_.setBitrateBufferSize(std::max(1,
            static_cast<int>(std::ceil(framerate * abr_component_.getBitrateTimeWindow()))));

        abr_component_.setVoterBufferSize(2);

        abr_component_.setStabilityTime(json_config.value("abr_stability_recover_time",
            abr_component_.getStabilityTime()));

        abr_component_.setCsv(json_config.value("csv", abr_component_.getCsv()));

        abr_component_.setStabilityThreshold(json_config.value("abr_stability_threshold",
            abr_component_.getStabilityThreshold()));

        abr_component_.setSimilarityThreshold(json_config.value("abr_similarity_threshold",
            abr_component_.getSimilarityThreshold()));

        abr_component_.setEmergencyTresh(json_config.value("abr_emergency_latency_threshold",
            abr_component_.getEmergencyTresh()));

        abr_component_.setPredictor(json_config.value("abr_predictor",
            abr_component_.getPredictor()));

        abr_component_.setRepublishData(json_config.value("republish_data",
            abr_component_.getRepublishData()));

        abr_component_.setRippleOrder(json_config.value("ripple_order",
            abr_component_.getRippleOrder()));

        abr_component_.reconfigureBuffers();

        auto system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        rclcpp::Time actual_time = system_clock->now();
        abr_component_.setStartTime(actual_time);
        abr_component_.setDateRefhesh(abr_component_.getStartTime());
        abr_component_.setPreviousTimeStamp(abr_component_.getStartTime());

        file1.close();

        abr_component_.initCsvFile();

        if (!bitrate_ladder.empty()) {
          abr_component_.bitrate_ladder = bitrate_ladder;
          double min_bitrate = *std::min_element(bitrate_ladder.begin(), bitrate_ladder.end());
          RCLCPP_INFO(logger_, "Minimum bitrate found by min_element: %f", min_bitrate);

          abr_component_.actual_bitrate = min_bitrate;
          RCLCPP_INFO(logger_, "Assigned minimum bitrate to actual_bitrate: %f",
              abr_component_.actual_bitrate);
        } else {
          RCLCPP_ERROR(logger_, "bitrate_ladder is empty, cannot set actual_bitrate.");
        }
        //Open gates
        allow_transmition_ = true; //Allows the codec to be opened
        abr_component_.allow_transmition_ = true; //Allows the Abr component to work


      } else {

        std::this_thread::sleep_for(std::chrono::seconds(1));
        abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket init_msg;
        init_msg.role = "client";
        init_msg.msg_type = 0;
        init_msg.msg_json = "{}";
        abr_info_publisher_->publish(init_msg);

        abr_component_.publish_msg_ =
          [this](const abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket & msg) {
            this->abr_info_publisher_->publish(msg);
          };

      }

    } else if (msg->msg_type == 1) {  //NEW CONFIG ASKED.

      nlohmann::json json_msg = nlohmann::json::parse(msg->msg_json);
      if (json_msg["confirmed"].get<bool>()) {

        //Positive confirmation from the publisher.
        decoder_.reset(); // Reset decoder to adapt to new configurations
        decoder_.resetTimers();

        double selected_bitrate = json_msg["selected_bitrate"].get<double>();
        abr_component_.previous_bitrate = abr_component_.actual_bitrate;
        abr_component_.actual_bitrate = selected_bitrate;  // Almacena como `double`


        RCLCPP_INFO(logger_, "Bitrate confirmed and updated: %f", selected_bitrate);

        allow_transmition_ = true;

        auto system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        rclcpp::Time actual_time = system_clock->now();

        abr_component_.setDateRefhesh(actual_time +
            rclcpp::Duration::from_seconds(abr_component_.getStabilityTime()));

        abr_component_.allow_transmition_ = true;


      } else {
        RCLCPP_WARN(logger_, "Bitrate change was not confirmed.");
      }

    } else if (msg->msg_type == 2) {
      RCLCPP_INFO(logger_, "TODO");

    } else {
      RCLCPP_INFO(logger_, "Wrong message type, ignoring msg");
    }
  }
}
}  // namespace abr_ffmpeg_image_transport
