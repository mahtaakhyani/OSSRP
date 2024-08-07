// Generated by gencpp from file infrastructure/TtsResponse.msg
// DO NOT EDIT!


#ifndef INFRASTRUCTURE_MESSAGE_TTSRESPONSE_H
#define INFRASTRUCTURE_MESSAGE_TTSRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <infrastructure/Tts_msg.h>

namespace infrastructure
{
template <class ContainerAllocator>
struct TtsResponse_
{
  typedef TtsResponse_<ContainerAllocator> Type;

  TtsResponse_()
    : speech()  {
    }
  TtsResponse_(const ContainerAllocator& _alloc)
    : speech(_alloc)  {
  (void)_alloc;
    }



   typedef  ::infrastructure::Tts_msg_<ContainerAllocator>  _speech_type;
  _speech_type speech;





  typedef boost::shared_ptr< ::infrastructure::TtsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::infrastructure::TtsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TtsResponse_

typedef ::infrastructure::TtsResponse_<std::allocator<void> > TtsResponse;

typedef boost::shared_ptr< ::infrastructure::TtsResponse > TtsResponsePtr;
typedef boost::shared_ptr< ::infrastructure::TtsResponse const> TtsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::infrastructure::TtsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::infrastructure::TtsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::infrastructure::TtsResponse_<ContainerAllocator1> & lhs, const ::infrastructure::TtsResponse_<ContainerAllocator2> & rhs)
{
  return lhs.speech == rhs.speech;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::infrastructure::TtsResponse_<ContainerAllocator1> & lhs, const ::infrastructure::TtsResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace infrastructure

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::TtsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::TtsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::infrastructure::TtsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::infrastructure::TtsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::TtsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::TtsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::infrastructure::TtsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2534803dfe8be7600047b56444e1d2de";
  }

  static const char* value(const ::infrastructure::TtsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2534803dfe8be760ULL;
  static const uint64_t static_value2 = 0x0047b56444e1d2deULL;
};

template<class ContainerAllocator>
struct DataType< ::infrastructure::TtsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "infrastructure/TtsResponse";
  }

  static const char* value(const ::infrastructure::TtsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::infrastructure::TtsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "infrastructure/Tts_msg speech\n"
"\n"
"================================================================================\n"
"MSG: infrastructure/Tts_msg\n"
"audio_common_msgs/AudioData[] data\n"
"================================================================================\n"
"MSG: audio_common_msgs/AudioData\n"
"uint8[] data\n"
;
  }

  static const char* value(const ::infrastructure::TtsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::infrastructure::TtsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.speech);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TtsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::infrastructure::TtsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::infrastructure::TtsResponse_<ContainerAllocator>& v)
  {
    s << indent << "speech: ";
    s << std::endl;
    Printer< ::infrastructure::Tts_msg_<ContainerAllocator> >::stream(s, indent + "  ", v.speech);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INFRASTRUCTURE_MESSAGE_TTSRESPONSE_H
