// Generated by gencpp from file infrastructure/GazeRequest.msg
// DO NOT EDIT!


#ifndef INFRASTRUCTURE_MESSAGE_GAZEREQUEST_H
#define INFRASTRUCTURE_MESSAGE_GAZEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <infrastructure/List.h>
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



   typedef  ::infrastructure::List_<ContainerAllocator>  _frame_type;
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
struct IsMessage< ::infrastructure::GazeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::infrastructure::GazeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::GazeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::GazeRequest_<ContainerAllocator> const>
  : FalseType
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
    return "563138297254f9391391e1e861953236";
  }

  static const char* value(const ::infrastructure::GazeRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x563138297254f939ULL;
  static const uint64_t static_value2 = 0x1391e1e861953236ULL;
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
    return "infrastructure/List  frame\n"
"infrastructure/Landmarks landmark\n"
"\n"
"================================================================================\n"
"MSG: infrastructure/List\n"
"infrastructure/Array3D[] list\n"
"================================================================================\n"
"MSG: infrastructure/Array3D\n"
"float64[] data\n"
"\n"
"================================================================================\n"
"MSG: infrastructure/Landmarks\n"
"string[] face\n"
"string[] left_hand\n"
"string[] right_hand\n"
"string[] pose\n"
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
    Printer< ::infrastructure::List_<ContainerAllocator> >::stream(s, indent + "  ", v.frame);
    s << indent << "landmark: ";
    s << std::endl;
    Printer< ::infrastructure::Landmarks_<ContainerAllocator> >::stream(s, indent + "  ", v.landmark);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INFRASTRUCTURE_MESSAGE_GAZEREQUEST_H
