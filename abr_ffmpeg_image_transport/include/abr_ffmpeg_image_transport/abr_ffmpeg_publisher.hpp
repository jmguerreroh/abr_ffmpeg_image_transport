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

#ifndef ABR_FFMPEG_IMAGE_TRANSPORT__ABR_FFMPEG_PUBLISHER_HPP_
#define ABR_FFMPEG_IMAGE_TRANSPORT__ABR_FFMPEG_PUBLISHER_HPP_

#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include "abr_ffmpeg_image_transport_interfaces/msg/abr_info_packet.hpp"
#include <image_transport/simple_publisher_plugin.hpp>
#include <memory>
#include <sensor_msgs/msg/image.hpp>
#include <nlohmann/json.hpp>


namespace abr_ffmpeg_image_transport
{
using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using FFMPEGPublisherPlugin = image_transport::SimplePublisherPlugin<FFMPEGPacket>;
using Image = sensor_msgs::msg::Image;
using FFMPEGPacketConstPtr = FFMPEGPacket::ConstSharedPtr;

class ABRFFMPEGPublisher : public FFMPEGPublisherPlugin
{
public:
  using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
  using ParameterValue = rclcpp::ParameterValue;
  struct ParameterDefinition
  {
    ParameterValue defaultValue;
    ParameterDescriptor descriptor;
  };

  ABRFFMPEGPublisher();
  ~ABRFFMPEGPublisher() override;
  std::string getTransportName() const override {return "abr_ffmpeg";}

protected:
#if defined(IMAGE_TRANSPORT_API_V1) || defined(IMAGE_TRANSPORT_API_V2)
  void advertiseImpl(
    rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos) override;
#else
  void advertiseImpl(
    rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos,
    rclcpp::PublisherOptions opt) override;
#endif
  void publish(const Image & message, const PublishFn & publish_fn) const override;


  bool init_ready = false;
  mutable bool ready_to_recive_video = false;

private:
  void packetReady(
    const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
    uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz);

  rmw_qos_profile_t initialize(
    rclcpp::Node * node, const std::string & base_name, rmw_qos_profile_t custom_qos);
  void declareParameter(
    rclcpp::Node * node, const std::string & base_name, const ParameterDefinition & definition);
  // variables ---------
  rclcpp::Logger logger_;
  const PublishFn * publishFunction_{NULL};
  ffmpeg_encoder_decoder::Encoder encoder_;
  uint32_t frameCounter_{0};
  // ---------- configurable parameters
  int performanceInterval_{175};  // num frames between perf printouts
  bool measurePerformance_{false};


  // ----- ABR -------
  // ABR communication channels
  rclcpp::Publisher<abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket>::SharedPtr
    abr_info_publisher_;
  rclcpp::Subscription<abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket>::SharedPtr
    abr_info_subscriber_;
  void abrInfoCallback(
    const abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket::SharedPtr msg);

  // Bitrate ladder
  mutable int width;
  mutable int height;
  mutable int forced_width;
  mutable int forced_height;
  mutable int selected_bitrate;
  rclcpp::Node * node_;

  mutable double framerate = 0.0;
  mutable rclcpp::Time framerate_ts;
  mutable std::vector<double> bitrate_ladder;
  mutable std::string bitrate_ladder_string = "";
  std::unordered_map<std::string, std::pair<int, int>> resolution_map_;

  mutable bool ladder_ready = false;

  mutable int framerate_sum = 0;     // Sum of framerate values in the last second
  mutable int framerate_count = 0;   // Number of values in the last second
  mutable rclcpp::Time last_update_time = rclcpp::Time(0, 0, RCL_STEADY_TIME);  // Last second timestamp


  std::string identifyResolution(int width, int height) const;
  void filterInefficientConfigurations(int width, int height) const;
  void filterResolutionsByName(const std::string & resolution_name, bool preserve) const;
  //std::pair<int, int> findBestResolutionMatch(int inner_width, int inner_height, int inner_selected_bitrate) const;
  std::pair<int, int> parseResolution(const std::string & res_str) const;
  void expectedBitrateLadder(int width, int height, int framerate) const;
  void filterBitrateLadder(double max_rate_mbps) const;

  // ----- ABR -------

};
}  // namespace abr_ffmpeg_image_transport

#endif  // ABR_FFMPEG_IMAGE_TRANSPORT__ABR_FFMPEG_PUBLISHER_HPP_
