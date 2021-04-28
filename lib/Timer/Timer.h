#ifndef TASKER_H
#define TASKER_H

#include "Arduino.h"

typedef void (*TimerCallback)();
typedef void (*TimerCallback1)(uintptr_t);

class Timeable {
public:
  virtual void loop() = 0;
};

struct Event {
  Timeable *timeable;
  TimerCallback1 callback;
  uintptr_t param;

  ulong interval;
  ulong time;

  int16_t counter;
  int16_t priority;

  Event *next;
  Event *prev;
};

class Timer {
public:
  Timer();
  virtual ~Timer();
  void loop();

  /**
   * Create a timeout that fires once after 'timeout' milliseconds
   */
  void setTimeout(Timeable *timeable, ulong timeout, int16_t priority = 0);
  void setTimeout(TimerCallback callback, ulong timeout, int16_t priority = 0);
  void setTimeout(TimerCallback1 callback, uintptr_t param, ulong timeout, int16_t priority = 0);

  /**
   * Create an interval that fires after each 'interval' milliseconds, repeat times (repeat < 0 == indefinitely)
   */
  void setInterval(Timeable *timeable, ulong interval, int16_t repeat = -1, int16_t priority = 0);
  void setInterval(TimerCallback callback, ulong interval, int16_t repeat = -1, int16_t priority = 0);
  void setInterval(TimerCallback1 callback, uintptr_t param, ulong interval, int16_t repeat = -1, int16_t priority = 0);


protected:
  Event *first;

  void addEvent(Event *event);
  void removeEvent(Event *event);
};

extern Timer Timer;

#endif