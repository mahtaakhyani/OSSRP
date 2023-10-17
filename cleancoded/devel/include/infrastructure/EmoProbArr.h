// Generated by gencpp from file infrastructure/EmoProbArr.msg
// DO NOT EDIT!


#ifndef INFRASTRUCTURE_MESSAGE_EMOPROBARR_H
#define INFRASTRUCTURE_MESSAGE_EMOPROBARR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace infrastructure
{
template <class ContainerAllocator>
struct EmoProbArr_
{
  typedef EmoProbArr_<ContainerAllocator> Type;

  EmoProbArr_()
    : emotion()
    , probability(0.0)  {
    }
  EmoProbArr_(const ContainerAllocator& _alloc)
    : emotion(_alloc)
    , probability(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _emotion_type;
  _emotion_type emotion;

   typedef float _probability_type;
  _probability_type probability;





  typedef boost::shared_ptr< ::infrastructure::EmoProbArr_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::infrastructure::EmoProbArr_<ContainerAllocator> const> ConstPtr;

}; // struct EmoProbArr_

typedef ::infrastructure::EmoProbArr_<std::allocator<void> > EmoProbArr;

typedef boost::shared_ptr< ::infrastructure::EmoProbArr > EmoProbArrPtr;
typedef boost::shared_ptr< ::infrastructure::EmoProbArr const> EmoProbArrConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::infrastructure::EmoProbArr_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::infrastructure::EmoProbArr_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::infrastructure::EmoProbArr_<ContainerAllocator1> & lhs, const ::infrastructure::EmoProbArr_<ContainerAllocator2> & rhs)
{
  return lhs.emotion == rhs.emotion &&
    lhs.probability == rhs.probability;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::infrastructure::EmoProbArr_<ContainerAllocator1> & lhs, const ::infrastructure::EmoProbArr_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace infrastructure

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::EmoProbArr_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::EmoProbArr_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::infrastructure::EmoProbArr_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::infrastructure::EmoProbArr_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::EmoProbArr_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::EmoProbArr_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::infrastructure::EmoProbArr_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a72bb7733492dee10dfc13b75d4a4cdd";
  }

  static const char* value(const ::infrastructure::EmoProbArr_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa72bb7733492dee1ULL;
  static const uint64_t static_value2 = 0x0dfc13b75d4a4cddULL;
};

template<class ContainerAllocator>
struct DataType< ::infrastructure::EmoProbArr_<ContainerAllocator> >
{
  static const char* value()
  {
    return "infrastructure/EmoProbArr";
  }

  static const char* value(const ::infrastructure::EmoProbArr_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::infrastructure::EmoProbArr_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string emotion\n"
"float32 probability\n"
;
  }

  static const char* value(const ::infrastructure::EmoProbArr_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::infrastructure::EmoProbArr_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.emotion);
      stream.next(m.probability);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EmoProbArr_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::infrastructure::EmoProbArr_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::infrastructure::EmoProbArr_<ContainerAllocator>& v)
  {
    s << indent << "emotion: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.emotion);
    s << indent << "probability: ";
    Printer<float>::stream(s, indent + "  ", v.probability);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INFRASTRUCTURE_MESSAGE_EMOPROBARR_H
