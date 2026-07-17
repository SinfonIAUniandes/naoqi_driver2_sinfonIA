/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef AUDIO_EVENT_REGISTER_HPP
#define AUDIO_EVENT_REGISTER_HPP

#include <cstdint>
#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <qi/session.hpp>

#include <rclcpp/rclcpp.hpp>
#include <audio_common_msgs/msg/audio_data_stamped.hpp>
#include <audio_common_msgs/msg/audio_data.hpp>
#include <audio_common_msgs/msg/audio_info.hpp>
#include <naoqi_driver/ros_helpers.hpp>

#include <naoqi_driver/tools.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

// Converter
#include "../src/converters/audio.hpp"
// Publisher
#include "../src/publishers/basic.hpp"
// Recorder
#include "../recorder/basic_event.hpp"

namespace naoqi
{

/**
* @brief GlobalRecorder concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/
class AudioEventRegister: public boost::enable_shared_from_this<AudioEventRegister>
{

public:

  /**
  * @brief Constructor for recorder interface
  */
  AudioEventRegister(
    const std::string& name,
    const std::string& stamped_name,
    const std::string& info_name,
    const float& frequency,
    const qi::SessionPtr& session,
    int sample_rate,
    int channel_config,
    bool deinterleaved,
    uint8_t channels,
    const std::string& sample_format,
    uint32_t bitrate,
    const std::string& coding_format );
  ~AudioEventRegister();

  void resetPublisher( rclcpp::Node* node );
  void resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr );

  void startProcess();
  void stopProcess();

  void writeDump(const rclcpp::Time& time);
  void setBufferDuration(float duration);

  void isRecording(bool state);
  void isPublishing(bool state);
  void isDumping(bool state);

  void processRemote(int nbOfChannels, int samplesByChannel, qi::AnyValue altimestamp, qi::AnyValue buffer);

private:
  void registerCallback();
  void unregisterCallback();
  void onEvent();

private:
  qi::SessionPtr session_;
  publisher::BasicPublisher<audio_common_msgs::msg::AudioData> audio_publisher_;
  publisher::BasicPublisher<audio_common_msgs::msg::AudioDataStamped> stamped_publisher_;
  rclcpp::Publisher<audio_common_msgs::msg::AudioInfo>::SharedPtr info_publisher_;
  recorder::BasicEventRecorder<audio_common_msgs::msg::AudioDataStamped> recorder_;
  converter::AudioEventConverter converter_;
  qi::AnyObject p_audio_;
  qi::FutureSync<qi::AnyObject> p_audio_extractor_request;
  std::vector<uint8_t> channelMap;
  unsigned int serviceId;
  std::string info_topic_;
  int sample_rate_;
  int channel_config_;
  bool deinterleaved_;
  audio_common_msgs::msg::AudioInfo audio_info_;

  boost::mutex subscription_mutex_;
  boost::mutex processing_mutex_;

  bool isStarted_;
  bool isPublishing_;
  bool isRecording_;
  bool isDumping_;

}; // class

QI_REGISTER_OBJECT(AudioEventRegister, processRemote)

} //naoqi

#endif
