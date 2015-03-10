/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/oscar/git/rover/catkin_ws/src/control_systems/msg/ArmAngles.msg
 *
 */


#ifndef CONTROL_SYSTEMS_MESSAGE_ARMANGLES_H
#define CONTROL_SYSTEMS_MESSAGE_ARMANGLES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace control_systems
{
template <class ContainerAllocator>
struct ArmAngles_
{
  typedef ArmAngles_<ContainerAllocator> Type;

  ArmAngles_()
    : shoulderOrientation(0.0)
    , shoulderElevation(0.0)
    , elbow(0.0)
    , wristOrientation(0.0)
    , wristElevation(0.0)  {
    }
  ArmAngles_(const ContainerAllocator& _alloc)
    : shoulderOrientation(0.0)
    , shoulderElevation(0.0)
    , elbow(0.0)
    , wristOrientation(0.0)
    , wristElevation(0.0)  {
    }



   typedef float _shoulderOrientation_type;
  _shoulderOrientation_type shoulderOrientation;

   typedef float _shoulderElevation_type;
  _shoulderElevation_type shoulderElevation;

   typedef float _elbow_type;
  _elbow_type elbow;

   typedef float _wristOrientation_type;
  _wristOrientation_type wristOrientation;

   typedef float _wristElevation_type;
  _wristElevation_type wristElevation;




  typedef boost::shared_ptr< ::control_systems::ArmAngles_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::control_systems::ArmAngles_<ContainerAllocator> const> ConstPtr;

}; // struct ArmAngles_

typedef ::control_systems::ArmAngles_<std::allocator<void> > ArmAngles;

typedef boost::shared_ptr< ::control_systems::ArmAngles > ArmAnglesPtr;
typedef boost::shared_ptr< ::control_systems::ArmAngles const> ArmAnglesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::control_systems::ArmAngles_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::control_systems::ArmAngles_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace control_systems

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'control_systems': ['/home/oscar/git/rover/catkin_ws/src/control_systems/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::control_systems::ArmAngles_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_systems::ArmAngles_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::control_systems::ArmAngles_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::control_systems::ArmAngles_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_systems::ArmAngles_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_systems::ArmAngles_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::control_systems::ArmAngles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5361e68128bb6d1538c253b719002d51";
  }

  static const char* value(const ::control_systems::ArmAngles_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5361e68128bb6d15ULL;
  static const uint64_t static_value2 = 0x38c253b719002d51ULL;
};

template<class ContainerAllocator>
struct DataType< ::control_systems::ArmAngles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "control_systems/ArmAngles";
  }

  static const char* value(const ::control_systems::ArmAngles_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::control_systems::ArmAngles_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 shoulderOrientation\n\
float32 shoulderElevation\n\
float32 elbow\n\
float32 wristOrientation\n\
float32 wristElevation\n\
";
  }

  static const char* value(const ::control_systems::ArmAngles_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::control_systems::ArmAngles_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.shoulderOrientation);
      stream.next(m.shoulderElevation);
      stream.next(m.elbow);
      stream.next(m.wristOrientation);
      stream.next(m.wristElevation);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct ArmAngles_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::control_systems::ArmAngles_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::control_systems::ArmAngles_<ContainerAllocator>& v)
  {
    s << indent << "shoulderOrientation: ";
    Printer<float>::stream(s, indent + "  ", v.shoulderOrientation);
    s << indent << "shoulderElevation: ";
    Printer<float>::stream(s, indent + "  ", v.shoulderElevation);
    s << indent << "elbow: ";
    Printer<float>::stream(s, indent + "  ", v.elbow);
    s << indent << "wristOrientation: ";
    Printer<float>::stream(s, indent + "  ", v.wristOrientation);
    s << indent << "wristElevation: ";
    Printer<float>::stream(s, indent + "  ", v.wristElevation);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROL_SYSTEMS_MESSAGE_ARMANGLES_H
