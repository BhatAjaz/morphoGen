namespace yarp iCub

enum Hand
{
  RIGHT = 0;
  LEFT=1;
  INDIFF=2;
}
service dmpExecutorInterface
{
  bool run();
  bool is_running();
  bool stop();
  bool execute_OPC(1:i32 id);
  bool waitMotionDone(1: double period, 2:double timeout);
  void set_hand(1:Hand newHand);
  Hand get_hand();
//  bool execute_DMP(1:DMP dmp);
}

