// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_RGBA
#define YARP_THRIFT_GENERATOR_STRUCT_RGBA

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace yarp {
  namespace sig {
    class RGBA;
  }
}


class yarp::sig::RGBA : public yarp::os::idl::WirePortable {
public:
  int32_t rgba;
  RGBA() : rgba(0) {
  }
  RGBA(const int32_t rgba) : rgba(rgba) {
  }
  bool read(yarp::os::idl::WireReader& reader) {
    if (!reader.readI32(rgba)) return false;
    return !reader.isError();
  }
  bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(1)) return false;
    return read(reader);
  }
  bool write(yarp::os::idl::WireWriter& writer) {
    if (!writer.writeI32(rgba)) return false;
    return !writer.isError();
  }
  bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    return write(writer);
  }
};

#endif

