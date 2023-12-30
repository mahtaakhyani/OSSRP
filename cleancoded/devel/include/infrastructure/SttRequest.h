// Generated by gencpp from file infrastructure/SttRequest.msg
// DO NOT EDIT!


#ifndef INFRASTRUCTURE_MESSAGE_STTREQUEST_H
#define INFRASTRUCTURE_MESSAGE_STTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <audio_common_msgs/AudioDataStamped.h>

namespace infrastructure
{
template <class ContainerAllocator>
struct SttRequest_
{
  typedef SttRequest_<ContainerAllocator> Type;

  SttRequest_()
    : audio()  {
    }
  SttRequest_(const ContainerAllocator& _alloc)
    : audio(_alloc)  {
  (void)_alloc;
    }



   typedef  ::audio_common_msgs::AudioDataStamped_<ContainerAllocator>  _audio_type;
  _audio_type audio;





  typedef boost::shared_ptr< ::infrastructure::SttRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::infrastructure::SttRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SttRequest_

typedef ::infrastructure::SttRequest_<std::allocator<void> > SttRequest;

typedef boost::shared_ptr< ::infrastructure::SttRequest > SttRequestPtr;
typedef boost::shared_ptr< ::infrastructure::SttRequest const> SttRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::infrastructure::SttRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::infrastructure::SttRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::infrastructure::SttRequest_<ContainerAllocator1> & lhs, const ::infrastructure::SttRequest_<ContainerAllocator2> & rhs)
{
  return lhs.audio == rhs.audio;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::infrastructure::SttRequest_<ContainerAllocator1> & lhs, const ::infrastructure::SttRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace infrastructure

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::SttRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::SttRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::infrastructure::SttRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::infrastructure::SttRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::SttRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::SttRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::infrastructure::SttRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "20c7f3b9bce053570f9d04bd783346e3";
  }

  static const char* value(const ::infrastructure::SttRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x20c7f3b9bce05357ULL;
  static const uint64_t static_value2 = 0x0f9d04bd783346e3ULL;
};

template<class ContainerAllocator>
struct DataType< ::infrastructure::SttRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "infrastructure/SttRequest";
  }

  static const char* value(const ::infrastructure::SttRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::infrastructure::SttRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "audio_common_msgs/AudioDataStamped audio\n"
"\n"
"================================================================================\n"
"MSG: audio_common_msgs/AudioDataStamped\n"
"std_msgs/Header header\n"
"audio_common_msgs/AudioData audio\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: audio_common_msgs/AudioData\n"
"uint8[] data\n"
;
  }

  static const char* value(const ::infrastructure::SttRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::infrastructure::SttRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.audio);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SttRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::infrastructure::SttRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::infrastructure::SttRequest_<ContainerAllocator>& v)
  {
    s << indent << "audio: ";
    s << std::endl;
    Printer< ::audio_common_msgs::AudioDataStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.audio);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INFRASTRUCTURE_MESSAGE_STTREQUEST_H
