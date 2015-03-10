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
 * Auto-generated by genmsg_cpp from file /home/oscar/git/rover/catkin_ws/src/control_systems/msg/SetPoints.msg
 *
 */


#ifndef CONTROL_SYSTEMS_MESSAGE_SETPOINTS_H
#define CONTROL_SYSTEMS_MESSAGE_SETPOINTS_H


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
struct SetPoints_
{
  typedef SetPoints_<ContainerAllocator> Type;

  SetPoints_()
    : thetaFL(0.0)
    , thetaFR(0.0)
    , thetaRL(0.0)
    , thetaRR(0.0)
    , speedFL(0.0)
    , speedFR(0.0)
    , speedML(0.0)
    , speedMR(0.0)
    , speedRL(0.0)
    , speedRR(0.0)  {
    }
  SetPoints_(const ContainerAllocator& _alloc)
    : thetaFL(0.0)
    , thetaFR(0.0)
    , thetaRL(0.0)
    , thetaRR(0.0)
    , speedFL(0.0)
    , speedFR(0.0)
    , speedML(0.0)
    , speedMR(0.0)
    , speedRL(0.0)
    , speedRR(0.0)  {
    }



   typedef float _thetaFL_type;
  _thetaFL_type thetaFL;

   typedef float _thetaFR_type;
  _thetaFR_type thetaFR;

   typedef float _thetaRL_type;
  _thetaRL_type thetaRL;

   typedef float _thetaRR_type;
  _thetaRR_type thetaRR;

   typedef float _speedFL_type;
  _speedFL_type speedFL;

   typedef float _speedFR_type;
  _speedFR_type speedFR;

   typedef float _speedML_type;
  _speedML_type speedML;

   typedef float _speedMR_type;
  _speedMR_type speedMR;

   typedef float _speedRL_type;
  _speedRL_type speedRL;

   typedef float _speedRR_type;
  _speedRR_type speedRR;




  typedef boost::shared_ptr< ::control_systems::SetPoints_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::control_systems::SetPoints_<ContainerAllocator> const> ConstPtr;

}; // struct SetPoints_

typedef ::control_systems::SetPoints_<std::allocator<void> > SetPoints;

typedef boost::shared_ptr< ::control_systems::SetPoints > SetPointsPtr;
typedef boost::shared_ptr< ::control_systems::SetPoints const> SetPointsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::control_systems::SetPoints_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::control_systems::SetPoints_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::control_systems::SetPoints_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_systems::SetPoints_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::control_systems::SetPoints_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::control_systems::SetPoints_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_systems::SetPoints_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_systems::SetPoints_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::control_systems::SetPoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8081cfe954eacee416d55d865a3b7a97";
  }

  static const char* value(const ::control_systems::SetPoints_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8081cfe954eacee4ULL;
  static const uint64_t static_value2 = 0x16d55d865a3b7a97ULL;
};

template<class ContainerAllocator>
struct DataType< ::control_systems::SetPoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "control_systems/SetPoints";
  }

  static const char* value(const ::control_systems::SetPoints_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::control_systems::SetPoints_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 thetaFL\n\
float32 thetaFR\n\
float32 thetaRL\n\
float32 thetaRR\n\
float32 speedFL\n\
float32 speedFR\n\
float32 speedML\n\
float32 speedMR\n\
float32 speedRL\n\
float32 speedRR\n\
";
  }

  static const char* value(const ::control_systems::SetPoints_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::control_systems::SetPoints_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.thetaFL);
      stream.next(m.thetaFR);
      stream.next(m.thetaRL);
      stream.next(m.thetaRR);
      stream.next(m.speedFL);
      stream.next(m.speedFR);
      stream.next(m.speedML);
      stream.next(m.speedMR);
      stream.next(m.speedRL);
      stream.next(m.speedRR);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct SetPoints_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::control_systems::SetPoints_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::control_systems::SetPoints_<ContainerAllocator>& v)
  {
    s << indent << "thetaFL: ";
    Printer<float>::stream(s, indent + "  ", v.thetaFL);
    s << indent << "thetaFR: ";
    Printer<float>::stream(s, indent + "  ", v.thetaFR);
    s << indent << "thetaRL: ";
    Printer<float>::stream(s, indent + "  ", v.thetaRL);
    s << indent << "thetaRR: ";
    Printer<float>::stream(s, indent + "  ", v.thetaRR);
    s << indent << "speedFL: ";
    Printer<float>::stream(s, indent + "  ", v.speedFL);
    s << indent << "speedFR: ";
    Printer<float>::stream(s, indent + "  ", v.speedFR);
    s << indent << "speedML: ";
    Printer<float>::stream(s, indent + "  ", v.speedML);
    s << indent << "speedMR: ";
    Printer<float>::stream(s, indent + "  ", v.speedMR);
    s << indent << "speedRL: ";
    Printer<float>::stream(s, indent + "  ", v.speedRL);
    s << indent << "speedRR: ";
    Printer<float>::stream(s, indent + "  ", v.speedRR);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROL_SYSTEMS_MESSAGE_SETPOINTS_H
