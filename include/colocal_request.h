// Generated by gencpp from file px4_csq/colocal_request.msg
// DO NOT EDIT!


#ifndef PX4_CSQ_MESSAGE_COLOCAL_REQUEST_H
#define PX4_CSQ_MESSAGE_COLOCAL_REQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace px4_csq
{
template <class ContainerAllocator>
struct colocal_request_
{
  typedef colocal_request_<ContainerAllocator> Type;

  colocal_request_()
    : id(0)
    , has_inited(false)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , yaw(0.0)  {
    }
  colocal_request_(const ContainerAllocator& _alloc)
    : id(0)
    , has_inited(false)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , yaw(0.0)  {
  (void)_alloc;
    }



   typedef int16_t _id_type;
  _id_type id;

   typedef uint8_t _has_inited_type;
  _has_inited_type has_inited;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;

   typedef double _yaw_type;
  _yaw_type yaw;





  typedef boost::shared_ptr< ::px4_csq::colocal_request_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::px4_csq::colocal_request_<ContainerAllocator> const> ConstPtr;

}; // struct colocal_request_

typedef ::px4_csq::colocal_request_<std::allocator<void> > colocal_request;

typedef boost::shared_ptr< ::px4_csq::colocal_request > colocal_requestPtr;
typedef boost::shared_ptr< ::px4_csq::colocal_request const> colocal_requestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::px4_csq::colocal_request_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::px4_csq::colocal_request_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace px4_csq

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'px4_csq': ['/home/jena/csq_ws/src/px4_csq/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::px4_csq::colocal_request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::px4_csq::colocal_request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::px4_csq::colocal_request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::px4_csq::colocal_request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::px4_csq::colocal_request_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::px4_csq::colocal_request_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::px4_csq::colocal_request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7590e61f48f016787f78f5bc054e2582";
  }

  static const char* value(const ::px4_csq::colocal_request_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7590e61f48f01678ULL;
  static const uint64_t static_value2 = 0x7f78f5bc054e2582ULL;
};

template<class ContainerAllocator>
struct DataType< ::px4_csq::colocal_request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "px4_csq/colocal_request";
  }

  static const char* value(const ::px4_csq::colocal_request_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::px4_csq::colocal_request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 id\n\
bool has_inited\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 yaw\n\
";
  }

  static const char* value(const ::px4_csq::colocal_request_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::px4_csq::colocal_request_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.has_inited);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.yaw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct colocal_request_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::px4_csq::colocal_request_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::px4_csq::colocal_request_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int16_t>::stream(s, indent + "  ", v.id);
    s << indent << "has_inited: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.has_inited);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PX4_CSQ_MESSAGE_COLOCAL_REQUEST_H
