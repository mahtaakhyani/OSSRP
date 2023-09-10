// Generated by gencpp from file infrastructure/List.msg
// DO NOT EDIT!


#ifndef INFRASTRUCTURE_MESSAGE_LIST_H
#define INFRASTRUCTURE_MESSAGE_LIST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <infrastructure/Array3D.h>

namespace infrastructure
{
template <class ContainerAllocator>
struct List_
{
  typedef List_<ContainerAllocator> Type;

  List_()
    : list()  {
    }
  List_(const ContainerAllocator& _alloc)
    : list(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::infrastructure::Array3D_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::infrastructure::Array3D_<ContainerAllocator> >> _list_type;
  _list_type list;





  typedef boost::shared_ptr< ::infrastructure::List_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::infrastructure::List_<ContainerAllocator> const> ConstPtr;

}; // struct List_

typedef ::infrastructure::List_<std::allocator<void> > List;

typedef boost::shared_ptr< ::infrastructure::List > ListPtr;
typedef boost::shared_ptr< ::infrastructure::List const> ListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::infrastructure::List_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::infrastructure::List_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::infrastructure::List_<ContainerAllocator1> & lhs, const ::infrastructure::List_<ContainerAllocator2> & rhs)
{
  return lhs.list == rhs.list;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::infrastructure::List_<ContainerAllocator1> & lhs, const ::infrastructure::List_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace infrastructure

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::infrastructure::List_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::infrastructure::List_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::List_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::List_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::List_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::List_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::infrastructure::List_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff61f15ae9ce7e4a9953c07b8d2a2270";
  }

  static const char* value(const ::infrastructure::List_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff61f15ae9ce7e4aULL;
  static const uint64_t static_value2 = 0x9953c07b8d2a2270ULL;
};

template<class ContainerAllocator>
struct DataType< ::infrastructure::List_<ContainerAllocator> >
{
  static const char* value()
  {
    return "infrastructure/List";
  }

  static const char* value(const ::infrastructure::List_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::infrastructure::List_<ContainerAllocator> >
{
  static const char* value()
  {
    return "infrastructure/Array3D[] list\n"
"================================================================================\n"
"MSG: infrastructure/Array3D\n"
"float64[] data\n"
;
  }

  static const char* value(const ::infrastructure::List_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::infrastructure::List_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.list);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct List_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::infrastructure::List_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::infrastructure::List_<ContainerAllocator>& v)
  {
    s << indent << "list[]" << std::endl;
    for (size_t i = 0; i < v.list.size(); ++i)
    {
      s << indent << "  list[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::infrastructure::Array3D_<ContainerAllocator> >::stream(s, indent + "    ", v.list[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // INFRASTRUCTURE_MESSAGE_LIST_H
