/* Auto-generated by genmsg_cpp for file /home/masaru/fuerte_workspace/sandbox/usarsim/usarsim_inf/msg/ToolchangerStatus.msg */
#ifndef USARSIM_INF_MESSAGE_TOOLCHANGERSTATUS_H
#define USARSIM_INF_MESSAGE_TOOLCHANGERSTATUS_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "usarsim_inf/EffectorStatus.h"
#include "usarsim_inf/ToolType.h"

namespace usarsim_inf
{
template <class ContainerAllocator>
struct ToolchangerStatus_ {
  typedef ToolchangerStatus_<ContainerAllocator> Type;

  ToolchangerStatus_()
  : header()
  , effector_status()
  , tool_type()
  {
  }

  ToolchangerStatus_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , effector_status(_alloc)
  , tool_type(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::usarsim_inf::EffectorStatus_<ContainerAllocator>  _effector_status_type;
   ::usarsim_inf::EffectorStatus_<ContainerAllocator>  effector_status;

  typedef  ::usarsim_inf::ToolType_<ContainerAllocator>  _tool_type_type;
   ::usarsim_inf::ToolType_<ContainerAllocator>  tool_type;


  typedef boost::shared_ptr< ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::usarsim_inf::ToolchangerStatus_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ToolchangerStatus
typedef  ::usarsim_inf::ToolchangerStatus_<std::allocator<void> > ToolchangerStatus;

typedef boost::shared_ptr< ::usarsim_inf::ToolchangerStatus> ToolchangerStatusPtr;
typedef boost::shared_ptr< ::usarsim_inf::ToolchangerStatus const> ToolchangerStatusConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace usarsim_inf

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::usarsim_inf::ToolchangerStatus_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8b5a06cc589e1ab2b44a5eec0fd708b9";
  }

  static const char* value(const  ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8b5a06cc589e1ab2ULL;
  static const uint64_t static_value2 = 0xb44a5eec0fd708b9ULL;
};

template<class ContainerAllocator>
struct DataType< ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "usarsim_inf/ToolchangerStatus";
  }

  static const char* value(const  ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
EffectorStatus effector_status\n\
ToolType tool_type\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: usarsim_inf/EffectorStatus\n\
Header header\n\
uint8 state\n\
uint8 OPEN=0\n\
uint8 CLOSE=1\n\
\n\
================================================================================\n\
MSG: usarsim_inf/ToolType\n\
uint8 type\n\
uint8 UNKNOWN=0\n\
uint8 GRIPPER=1\n\
uint8 VACUUM=2\n\
uint8 TOOLCHANGER=3\n\
\n\
";
  }

  static const char* value(const  ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.effector_status);
    stream.next(m.tool_type);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ToolchangerStatus_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::usarsim_inf::ToolchangerStatus_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "effector_status: ";
s << std::endl;
    Printer< ::usarsim_inf::EffectorStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.effector_status);
    s << indent << "tool_type: ";
s << std::endl;
    Printer< ::usarsim_inf::ToolType_<ContainerAllocator> >::stream(s, indent + "  ", v.tool_type);
  }
};


} // namespace message_operations
} // namespace ros

#endif // USARSIM_INF_MESSAGE_TOOLCHANGERSTATUS_H

