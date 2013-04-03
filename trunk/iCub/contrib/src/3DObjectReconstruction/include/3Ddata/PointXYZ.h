// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_PointXYZ
#define YARP_THRIFT_GENERATOR_STRUCT_PointXYZ

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace yarp {
  namespace sig {
    class PointXYZ;
  }
}


class yarp::sig::PointXYZ : public yarp::os::idl::WirePortable {
public:
  double x;
  double y;
  double z;
  PointXYZ() : x(0), y(0), z(0) {
  }
  PointXYZ(const double x,const double y,const double z) : x(x), y(y), z(z) {
  }
  bool read(yarp::os::idl::WireReader& reader) {
    if (!reader.readDouble(x)) return false;
    if (!reader.readDouble(y)) return false;
    if (!reader.readDouble(z)) return false;
    return !reader.isError();
  }
  bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(3)) return false;
    return read(reader);
  }
  bool write(yarp::os::idl::WireWriter& writer) {
    if (!writer.writeDouble(x)) return false;
    if (!writer.writeDouble(y)) return false;
    if (!writer.writeDouble(z)) return false;
    return !writer.isError();
  }
  bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    return write(writer);
  }
};

#endif

