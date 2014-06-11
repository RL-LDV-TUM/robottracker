#ifndef _RobotMsg_H_
#define _RobotMsg_H_

/*
* Protocol Message for Robottracker
*/
#pragma pack(1)
struct RobotMsg
{ 
  // robot marker id
  uint32_t id;
  
  // last seen X seconds ago
  int32_t sec;
  
  // coordinates relative to Playground
  float x;
  float y;
  // just for fun, if robot rises ;-)
  float z; 
  
  // rounded to cell (x:[1-10], y:[1-7]) 
  uint32_t xCell;
  uint32_t yCell;
  
  float angle;
};
#pragma pack()




/*
* Some helper functions to handle RobotMessages
*/
namespace RobotMessage
{

  void print(const RobotMsg *msg);
  void hton(RobotMsg *msg);
  void ntoh(RobotMsg *msg);

}

#endif
