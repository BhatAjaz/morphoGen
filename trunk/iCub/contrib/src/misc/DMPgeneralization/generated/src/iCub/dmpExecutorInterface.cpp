// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <iCub/dmpExecutorInterface.h>
#include <yarp/os/idl/WireTypes.h>

namespace iCub {


class dmpExecutorInterface_run : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("run",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) return false;
    return true;
  }
};

class dmpExecutorInterface_is_running : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("is_running",1,2)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) return false;
    return true;
  }
};

class dmpExecutorInterface_stop : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("stop",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) return false;
    return true;
  }
};

class dmpExecutorInterface_execute_OPC : public yarp::os::Portable {
public:
  int32_t id;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("execute_OPC",1,2)) return false;
    if (!writer.writeI32(id)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) return false;
    return true;
  }
};

class dmpExecutorInterface_waitMotionDone : public yarp::os::Portable {
public:
  double period;
  double timeout;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("waitMotionDone",1,1)) return false;
    if (!writer.writeDouble(period)) return false;
    if (!writer.writeDouble(timeout)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) return false;
    return true;
  }
};

class dmpExecutorInterface_set_hand : public yarp::os::Portable {
public:
  Hand newHand;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("set_hand",1,2)) return false;
    if (!writer.writeI32((int32_t)newHand)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    return true;
  }
};

class dmpExecutorInterface_get_hand : public yarp::os::Portable {
public:
  Hand _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("get_hand",1,2)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    int32_t ecast0;
    HandVocab cvrt1;
    if (!reader.readEnum(ecast0,cvrt1)) return false;
    _return = (Hand)ecast0;
    return true;
  }
};

bool dmpExecutorInterface::run() {
  bool _return = false;
  dmpExecutorInterface_run helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool dmpExecutorInterface::is_running() {
  bool _return = false;
  dmpExecutorInterface_is_running helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool dmpExecutorInterface::stop() {
  bool _return = false;
  dmpExecutorInterface_stop helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool dmpExecutorInterface::execute_OPC(const int32_t id) {
  bool _return = false;
  dmpExecutorInterface_execute_OPC helper;
  helper.id = id;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool dmpExecutorInterface::waitMotionDone(const double period, const double timeout) {
  bool _return = false;
  dmpExecutorInterface_waitMotionDone helper;
  helper.period = period;
  helper.timeout = timeout;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
void dmpExecutorInterface::set_hand(const Hand newHand) {
  dmpExecutorInterface_set_hand helper;
  helper.newHand = newHand;
  yarp().write(helper,helper);
}
Hand dmpExecutorInterface::get_hand() {
  Hand _return = (Hand)0;
  dmpExecutorInterface_get_hand helper;
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool dmpExecutorInterface::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) return false;
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {    // TODO: use quick lookup, this is just a test
    if (tag == "run") {
      bool _return;
      _return = run();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "is_running") {
      bool _return;
      _return = is_running();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "stop") {
      bool _return;
      _return = stop();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "execute_OPC") {
      int32_t id;
      if (!reader.readI32(id)) return false;
      bool _return;
      _return = execute_OPC(id);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "waitMotionDone") {
      double period;
      double timeout;
      if (!reader.readDouble(period)) return false;
      if (!reader.readDouble(timeout)) return false;
      bool _return;
      _return = waitMotionDone(period,timeout);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_hand") {
      Hand newHand;
      int32_t ecast2;
      HandVocab cvrt3;
      if (!reader.readEnum(ecast2,cvrt3)) return false;
      newHand = (Hand)ecast2;
      set_hand(newHand);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_hand") {
      Hand _return;
      _return = get_hand();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeI32((int32_t)_return)) return false;
      }
      reader.accept();
      return true;
    }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}
} // namespace


