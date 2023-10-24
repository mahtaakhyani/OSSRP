// Generated by gencpp from file infrastructure/Tts.msg
// DO NOT EDIT!


#ifndef INFRASTRUCTURE_MESSAGE_TTS_H
#define INFRASTRUCTURE_MESSAGE_TTS_H

#include <ros/service_traits.h>


#include <infrastructure/TtsRequest.h>
#include <infrastructure/TtsResponse.h>


namespace infrastructure
{

struct Tts
{

typedef TtsRequest Request;
typedef TtsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Tts
} // namespace infrastructure


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::infrastructure::Tts > {
  static const char* value()
  {
    return "32b04ab27e111935c7e979243abf2412";
  }

  static const char* value(const ::infrastructure::Tts&) { return value(); }
};

template<>
struct DataType< ::infrastructure::Tts > {
  static const char* value()
  {
    return "infrastructure/Tts";
  }

  static const char* value(const ::infrastructure::Tts&) { return value(); }
};


// service_traits::MD5Sum< ::infrastructure::TtsRequest> should match
// service_traits::MD5Sum< ::infrastructure::Tts >
template<>
struct MD5Sum< ::infrastructure::TtsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::infrastructure::Tts >::value();
  }
  static const char* value(const ::infrastructure::TtsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::infrastructure::TtsRequest> should match
// service_traits::DataType< ::infrastructure::Tts >
template<>
struct DataType< ::infrastructure::TtsRequest>
{
  static const char* value()
  {
    return DataType< ::infrastructure::Tts >::value();
  }
  static const char* value(const ::infrastructure::TtsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::infrastructure::TtsResponse> should match
// service_traits::MD5Sum< ::infrastructure::Tts >
template<>
struct MD5Sum< ::infrastructure::TtsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::infrastructure::Tts >::value();
  }
  static const char* value(const ::infrastructure::TtsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::infrastructure::TtsResponse> should match
// service_traits::DataType< ::infrastructure::Tts >
template<>
struct DataType< ::infrastructure::TtsResponse>
{
  static const char* value()
  {
    return DataType< ::infrastructure::Tts >::value();
  }
  static const char* value(const ::infrastructure::TtsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // INFRASTRUCTURE_MESSAGE_TTS_H
