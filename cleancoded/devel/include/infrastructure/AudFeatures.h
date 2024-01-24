// Generated by gencpp from file infrastructure/AudFeatures.msg
// DO NOT EDIT!


#ifndef INFRASTRUCTURE_MESSAGE_AUDFEATURES_H
#define INFRASTRUCTURE_MESSAGE_AUDFEATURES_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace infrastructure
{
template <class ContainerAllocator>
struct AudFeatures_
{
  typedef AudFeatures_<ContainerAllocator> Type;

  AudFeatures_()
    : min_f0(0.0)
    , max_f0(0.0)
    , mean_f0(0.0)
    , min_int(0.0)
    , max_int(0.0)
    , mean_int(0.0)
    , jitter(0.0)
    , shimmer(0.0)
    , hnr(0.0)
    , speaking_rate(0.0)  {
    }
  AudFeatures_(const ContainerAllocator& _alloc)
    : min_f0(0.0)
    , max_f0(0.0)
    , mean_f0(0.0)
    , min_int(0.0)
    , max_int(0.0)
    , mean_int(0.0)
    , jitter(0.0)
    , shimmer(0.0)
    , hnr(0.0)
    , speaking_rate(0.0)  {
  (void)_alloc;
    }



   typedef double _min_f0_type;
  _min_f0_type min_f0;

   typedef double _max_f0_type;
  _max_f0_type max_f0;

   typedef double _mean_f0_type;
  _mean_f0_type mean_f0;

   typedef double _min_int_type;
  _min_int_type min_int;

   typedef double _max_int_type;
  _max_int_type max_int;

   typedef double _mean_int_type;
  _mean_int_type mean_int;

   typedef double _jitter_type;
  _jitter_type jitter;

   typedef double _shimmer_type;
  _shimmer_type shimmer;

   typedef double _hnr_type;
  _hnr_type hnr;

   typedef double _speaking_rate_type;
  _speaking_rate_type speaking_rate;





  typedef boost::shared_ptr< ::infrastructure::AudFeatures_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::infrastructure::AudFeatures_<ContainerAllocator> const> ConstPtr;

}; // struct AudFeatures_

typedef ::infrastructure::AudFeatures_<std::allocator<void> > AudFeatures;

typedef boost::shared_ptr< ::infrastructure::AudFeatures > AudFeaturesPtr;
typedef boost::shared_ptr< ::infrastructure::AudFeatures const> AudFeaturesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::infrastructure::AudFeatures_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::infrastructure::AudFeatures_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::infrastructure::AudFeatures_<ContainerAllocator1> & lhs, const ::infrastructure::AudFeatures_<ContainerAllocator2> & rhs)
{
  return lhs.min_f0 == rhs.min_f0 &&
    lhs.max_f0 == rhs.max_f0 &&
    lhs.mean_f0 == rhs.mean_f0 &&
    lhs.min_int == rhs.min_int &&
    lhs.max_int == rhs.max_int &&
    lhs.mean_int == rhs.mean_int &&
    lhs.jitter == rhs.jitter &&
    lhs.shimmer == rhs.shimmer &&
    lhs.hnr == rhs.hnr &&
    lhs.speaking_rate == rhs.speaking_rate;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::infrastructure::AudFeatures_<ContainerAllocator1> & lhs, const ::infrastructure::AudFeatures_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace infrastructure

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::infrastructure::AudFeatures_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::infrastructure::AudFeatures_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::AudFeatures_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::infrastructure::AudFeatures_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::AudFeatures_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::infrastructure::AudFeatures_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::infrastructure::AudFeatures_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7c6ab6a6d7305a866c023967280bedf4";
  }

  static const char* value(const ::infrastructure::AudFeatures_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7c6ab6a6d7305a86ULL;
  static const uint64_t static_value2 = 0x6c023967280bedf4ULL;
};

template<class ContainerAllocator>
struct DataType< ::infrastructure::AudFeatures_<ContainerAllocator> >
{
  static const char* value()
  {
    return "infrastructure/AudFeatures";
  }

  static const char* value(const ::infrastructure::AudFeatures_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::infrastructure::AudFeatures_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 min_f0 #minimum fundamental frequency\n"
"float64 max_f0 #maximum fundamental frequency\n"
"float64 mean_f0 #mean fundamental frequency\n"
"float64 min_int #minimum intensity\n"
"float64 max_int #maximum intensity\n"
"float64 mean_int #mean intensity\n"
"float64 jitter #local\n"
"float64 shimmer #local\n"
"float64 hnr #harmonic to noise ratio\n"
"float64 speaking_rate #of syllables per second\n"
;
  }

  static const char* value(const ::infrastructure::AudFeatures_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::infrastructure::AudFeatures_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.min_f0);
      stream.next(m.max_f0);
      stream.next(m.mean_f0);
      stream.next(m.min_int);
      stream.next(m.max_int);
      stream.next(m.mean_int);
      stream.next(m.jitter);
      stream.next(m.shimmer);
      stream.next(m.hnr);
      stream.next(m.speaking_rate);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AudFeatures_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::infrastructure::AudFeatures_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::infrastructure::AudFeatures_<ContainerAllocator>& v)
  {
    s << indent << "min_f0: ";
    Printer<double>::stream(s, indent + "  ", v.min_f0);
    s << indent << "max_f0: ";
    Printer<double>::stream(s, indent + "  ", v.max_f0);
    s << indent << "mean_f0: ";
    Printer<double>::stream(s, indent + "  ", v.mean_f0);
    s << indent << "min_int: ";
    Printer<double>::stream(s, indent + "  ", v.min_int);
    s << indent << "max_int: ";
    Printer<double>::stream(s, indent + "  ", v.max_int);
    s << indent << "mean_int: ";
    Printer<double>::stream(s, indent + "  ", v.mean_int);
    s << indent << "jitter: ";
    Printer<double>::stream(s, indent + "  ", v.jitter);
    s << indent << "shimmer: ";
    Printer<double>::stream(s, indent + "  ", v.shimmer);
    s << indent << "hnr: ";
    Printer<double>::stream(s, indent + "  ", v.hnr);
    s << indent << "speaking_rate: ";
    Printer<double>::stream(s, indent + "  ", v.speaking_rate);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INFRASTRUCTURE_MESSAGE_AUDFEATURES_H
