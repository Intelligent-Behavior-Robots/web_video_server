#include "web_video_server/h264_streamer.h"


namespace web_video_server
{

H264Streamer::H264Streamer(const async_web_server_cpp::HttpRequest& request,
                         async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr nh) :
    LibavStreamer(request, connection, nh, "mp4", "libx264", "video/mp4")
{
  /* possible quality presets:
   * ultrafast, superfast, veryfast, faster, fast, medium, slow, slower, veryslow, placebo
   * no latency improvements observed with ultrafast instead of medium
   */
  preset_ = request.get_query_param_value_or_default("preset", "faster");
}

H264Streamer::~H264Streamer()
{
}

void H264Streamer::initializeEncoder()
{

  codec_context_->bit_rate = 150000;
  codec_context_->time_base = {1, 15};
  codec_context_->framerate = {15, 1};
  codec_context_->gop_size = 1;
  codec_context_->max_b_frames = 0;
  codec_context_->delay = 0;
  codec_context_->thread_count = 1;
  codec_context_->width = 1920;
  codec_context_->height = 1080;



  

  av_opt_set(codec_context_->priv_data, "preset", preset_.c_str(), 0);
  av_opt_set(codec_context_->priv_data, "tune", "zerolatency", 0);
  av_opt_set_int(codec_context_->priv_data, "crf", 20, 0);
  av_opt_set_int(codec_context_->priv_data, "bufsize", 3000, 0);
  av_opt_set_int(codec_context_->priv_data, "keyint", 1, 0);
  av_opt_set_int(codec_context_->priv_data, "g", 1, 0);
    


  

  // container format options
  if (!strcmp(format_context_->oformat->name, "mp4")) {
    // set up mp4 for streaming (instead of seekable file output)
    av_dict_set(&opt_, "movflags", "+frag_keyframe+empty_moov+faststart", 0);
  }
}

H264StreamerType::H264StreamerType() :
    LibavStreamerType("mp4", "libx264", "video/mp4")
{
}

boost::shared_ptr<ImageStreamer> H264StreamerType::create_streamer(const async_web_server_cpp::HttpRequest& request,
                                                                  async_web_server_cpp::HttpConnectionPtr connection,
                                                                  rclcpp::Node::SharedPtr nh)
{
  return boost::shared_ptr<ImageStreamer>(new H264Streamer(request, connection, nh));
}

}
