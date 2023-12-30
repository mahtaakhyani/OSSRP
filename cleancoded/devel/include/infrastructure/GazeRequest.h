// Generated by gencpp from file infrastructure/GazeRequest.msg
// DO NOT EDIT!


#ifndef INFRASTRUCTURE_MESSAGE_GAZEREQUEST_H
#define INFRASTRUCTURE_MESSAGE_GAZEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/Image.h>
#include <infrastructure/Landmarks.h>

namespace infrastructure
{
template <class ContainerAllocator>
struct GazeRequest_
{
  typedef GazeRequest_<ContainerAllocator> Type;

  GazeRequest_()
    : frame()
    , landmark()  {
    }
  GazeRequest_(const ContainerAllocator& _alloc)
    : frame(_alloc)
    , landmark(_alloc)  {
  (void)_alloc;
    }



   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _frame_type;
  _frame_type frame;

   typedef  ::infrastructure::Landmarks_<ContainerAllocator>  _landmark_type;
  _landmark_type landmark;





  typedef boost::shared_ptr< ::infrastructure::GazeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::infrastructure::GazeRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GazeRequest_

typedef ::infrastructure::GazeRequest_<std::allocator<void> > GazeRequest;

typedef boost::shared_ptr< ::infrastructure::GazeRequest > GazeRequestPtr;
typedef boost::shared_ptr< ::infrastructure::GazeRequest const> GazeRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::infrastructure::GazeRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::infrastructure::GazeRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::infrastructure::GazeRequest_<ContainerAllocator1> & lhs, const ::infrastructure::GazeRequest_<ContainerAllocator2> & rhs)
{
  return lhs.frame == rhs.frame &&
    lhs.landmark == rhs.landmark;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::infrastructure::GazeRequest_<ContainerAllocator1> & lhs, const ::infrastructure::GazeRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace infrastructure

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::GazeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::GazeRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::infrastructure::GazeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::infrastructure::GazeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::GazeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::GazeRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::infrastructure::GazeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c1fe2cc637bb12c3915e2dc16eb8797a";
  }

  static const char* value(const ::infrastructure::GazeRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc1fe2cc637bb12c3ULL;
  static const uint64_t static_value2 = 0x915e2dc16eb8797aULL;
};

template<class ContainerAllocator>
struct DataType< ::infrastructure::GazeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "infrastructure/GazeRequest";
  }

  static const char* value(const ::infrastructure::GazeRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::infrastructure::GazeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_msgs/Image  frame\n"
"infrastructure/Landmarks landmark\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/Image\n"
"# This message contains an uncompressed image\n"
"# (0, 0) is at top-left corner of image\n"
"#\n"
"\n"
"Header header        # Header timestamp should be acquisition time of image\n"
"                     # Header frame_id should be optical frame of camera\n"
"                     # origin of frame should be optical center of camera\n"
"                     # +x should point to the right in the image\n"
"                     # +y should point down in the image\n"
"                     # +z should point into to plane of the image\n"
"                     # If the frame_id here and the frame_id of the CameraInfo\n"
"                     # message associated with the image conflict\n"
"                     # the behavior is undefined\n"
"\n"
"uint32 height         # image height, that is, number of rows\n"
"uint32 width          # image width, that is, number of columns\n"
"\n"
"# The legal values for encoding are in file src/image_encodings.cpp\n"
"# If you want to standardize a new string format, join\n"
"# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n"
"\n"
"string encoding       # Encoding of pixels -- channel meaning, ordering, size\n"
"                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n"
"\n"
"uint8 is_bigendian    # is this data bigendian?\n"
"uint32 step           # Full row length in bytes\n"
"uint8[] data          # actual matrix data, size is (step * rows)\n"
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
"MSG: infrastructure/Landmarks\n"
"geometry_msgs/Point[] face\n"
"geometry_msgs/Point[] left_hand\n"
"geometry_msgs/Point[] right_hand\n"
"geometry_msgs/Point[] pose\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::infrastructure::GazeRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::infrastructure::GazeRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.frame);
      stream.next(m.landmark);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GazeRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::infrastructure::GazeRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::infrastructure::GazeRequest_<ContainerAllocator>& v)
  {
    s << indent << "frame: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.frame);
    s << indent << "landmark: ";
    s << std::endl;
    Printer< ::infrastructure::Landmarks_<ContainerAllocator> >::stream(s, indent + "  ", v.landmark);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INFRASTRUCTURE_MESSAGE_GAZEREQUEST_H
