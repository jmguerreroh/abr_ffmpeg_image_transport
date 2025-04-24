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

#ifndef ABR_FFMPEG_IMAGE_TRANSPORT__ABR_FFMPEG_SUBSCRIBER_HPP_
#define ABR_FFMPEG_IMAGE_TRANSPORT__ABR_FFMPEG_SUBSCRIBER_HPP_

#include <ffmpeg_encoder_decoder/decoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <image_transport/simple_subscriber_plugin.hpp>
#include <string>

#include "abr_ffmpeg_image_transport/abr_component.hpp"

namespace abr_ffmpeg_image_transport
{
using Image = sensor_msgs::msg::Image;
using ImageConstPtr = Image::ConstSharedPtr;
using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using FFMPEGPacketConstPtr = FFMPEGPacket::ConstSharedPtr;
using FFMPEGSubscriberPlugin = image_transport::SimpleSubscriberPlugin<FFMPEGPacket>;

class ABRFFMPEGSubscriber : public FFMPEGSubscriberPlugin
{
public:
  ABRFFMPEGSubscriber();
  ~ABRFFMPEGSubscriber();

  std::string getTransportName() const override {return "abr_ffmpeg";}

protected:
  void internalCallback(const FFMPEGPacketConstPtr & msg, const Callback & user_cb) override;

#ifdef IMAGE_TRANSPORT_API_V1
  void subscribeImpl(
    rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
    rmw_qos_profile_t custom_qos) override;
#else
  void subscribeImpl(
    rclcpp::Node * node, const std::string & base_topic, const Callback & callback,
    rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions) override;
#endif


  int desired_width;
  int desired_height;
  int framerate;
  rclcpp::Node * node_;

private:
  void frameReady(const ImageConstPtr & img, bool /*isKeyFrame*/) const;
  void initialize(rclcpp::Node * node, const std::string & base_topics);
  // -------------- variables
  rclcpp::Logger logger_;
  ffmpeg_encoder_decoder::Decoder decoder_;
  std::string decoderType_;
  const Callback * userCallback_;
  std::string param_namespace_;


  //ABR algorithm
  AbrComponent abr_component_;
  bool allow_transmition_ = false;
  mutable std::vector<double> bitrate_ladder;

  // ABR communication channels
  rclcpp::Publisher<abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket>::SharedPtr
    abr_info_publisher_;
  rclcpp::Subscription<abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket>::SharedPtr
    abr_info_subscriber_;
  void abrInfoCallback(
    const abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket::SharedPtr msg);

};
}  // namespace abr_ffmpeg_image_transport
#endif  // ABR_FFMPEG_IMAGE_TRANSPORT__ABR_FFMPEG_SUBSCRIBER_HPP_
