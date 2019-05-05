// Generated by gencpp from file px4_csq/frame_ros.msg
// DO NOT EDIT!


#ifndef PX4_CSQ_MESSAGE_FRAME_ROS_H
#define PX4_CSQ_MESSAGE_FRAME_ROS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace px4_csq
{
template <class ContainerAllocator>
struct frame_ros_
{
  typedef frame_ros_<ContainerAllocator> Type;

  frame_ros_()
    : header()
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , qw(0.0)
    , qx(0.0)
    , qy(0.0)
    , qz(0.0)
    , nrobotid(0)
    , data()
    , mDepth()
    , mvuRight()  {
    }
  frame_ros_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , qw(0.0)
    , qx(0.0)
    , qy(0.0)
    , qz(0.0)
    , nrobotid(0)
    , data(_alloc)
    , mDepth(_alloc)
    , mvuRight(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _qw_type;
  _qw_type qw;

   typedef float _qx_type;
  _qx_type qx;

   typedef float _qy_type;
  _qy_type qy;

   typedef float _qz_type;
  _qz_type qz;

   typedef int16_t _nrobotid_type;
  _nrobotid_type nrobotid;

   typedef std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  _data_type;
  _data_type data;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _mDepth_type;
  _mDepth_type mDepth;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _mvuRight_type;
  _mvuRight_type mvuRight;





  typedef boost::shared_ptr< ::px4_csq::frame_ros_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::px4_csq::frame_ros_<ContainerAllocator> const> ConstPtr;

}; // struct frame_ros_

typedef ::px4_csq::frame_ros_<std::allocator<void> > frame_ros;

typedef boost::shared_ptr< ::px4_csq::frame_ros > frame_rosPtr;
typedef boost::shared_ptr< ::px4_csq::frame_ros const> frame_rosConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::px4_csq::frame_ros_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::px4_csq::frame_ros_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace px4_csq

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'px4_csq': ['/home/jena/csq_ws/src/px4_csq/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::px4_csq::frame_ros_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::px4_csq::frame_ros_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::px4_csq::frame_ros_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::px4_csq::frame_ros_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::px4_csq::frame_ros_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::px4_csq::frame_ros_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::px4_csq::frame_ros_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6fa8ffce8c665540351073d063d8deef";
  }

  static const char* value(const ::px4_csq::frame_ros_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6fa8ffce8c665540ULL;
  static const uint64_t static_value2 = 0x351073d063d8deefULL;
};

template<class ContainerAllocator>
struct DataType< ::px4_csq::frame_ros_<ContainerAllocator> >
{
  static const char* value()
  {
    return "px4_csq/frame_ros";
  }

  static const char* value(const ::px4_csq::frame_ros_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::px4_csq::frame_ros_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float32 x\n\
float32 y\n\
float32 z\n\
float32 qw\n\
float32 qx\n\
float32 qy\n\
float32 qz\n\
int16 nrobotid\n\
byte[] data\n\
float32[] mDepth\n\
float32[] mvuRight\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::px4_csq::frame_ros_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::px4_csq::frame_ros_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.qw);
      stream.next(m.qx);
      stream.next(m.qy);
      stream.next(m.qz);
      stream.next(m.nrobotid);
      stream.next(m.data);
      stream.next(m.mDepth);
      stream.next(m.mvuRight);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct frame_ros_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::px4_csq::frame_ros_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::px4_csq::frame_ros_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "qw: ";
    Printer<float>::stream(s, indent + "  ", v.qw);
    s << indent << "qx: ";
    Printer<float>::stream(s, indent + "  ", v.qx);
    s << indent << "qy: ";
    Printer<float>::stream(s, indent + "  ", v.qy);
    s << indent << "qz: ";
    Printer<float>::stream(s, indent + "  ", v.qz);
    s << indent << "nrobotid: ";
    Printer<int16_t>::stream(s, indent + "  ", v.nrobotid);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.data[i]);
    }
    s << indent << "mDepth[]" << std::endl;
    for (size_t i = 0; i < v.mDepth.size(); ++i)
    {
      s << indent << "  mDepth[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.mDepth[i]);
    }
    s << indent << "mvuRight[]" << std::endl;
    for (size_t i = 0; i < v.mvuRight.size(); ++i)
    {
      s << indent << "  mvuRight[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.mvuRight[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PX4_CSQ_MESSAGE_FRAME_ROS_H