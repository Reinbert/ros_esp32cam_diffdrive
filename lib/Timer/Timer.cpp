#include "Timer.h"

Timer::Timer() : first(nullptr) {

}

Timer::~Timer() {
  first = nullptr;
}






// Create a timeout that fires once after 'timeout' milliseconds
// ------------------------------------------------------------------------------------------------
void Timer::setTimeout(Timeable *timeable, ulong timeout, int16_t priority) {
  setInterval(timeable, timeout, 1, priority);
}

void Timer::setTimeout(TimerCallback callback, ulong timeout, int16_t priority) {
  setInterval(callback, timeout, 1, priority);
}

void Timer::setTimeout(TimerCallback1 callback, uintptr_t param, ulong timeout, int16_t priority) {
  setInterval((TimerCallback1) callback, param, timeout, 1, priority);
}

// Create an interval that fires after each 'interval' milliseconds, repeat times (repeat < 0 == indefinitely)
// ------------------------------------------------------------------------------------------------
void Timer::setInterval(Timeable *timeable, ulong interval, int16_t repeat, int16_t priority) {
  // 'repeat' needs to be != 0 for an event to be created
  if (repeat) {
    Event *event = new Event();
    event->timeable = timeable;
    event->param = 0;
    event->interval = interval;
    event->time = interval + millis();
    event->counter = repeat;
    event->priority = priority;
    addEvent(event);
  }
}

void Timer::setInterval(TimerCallback callback, ulong interval, int16_t repeat, int16_t priority) {
  setInterval((TimerCallback1) callback, 0, interval, repeat, priority);
}

void Timer::setInterval(TimerCallback1 callback, uintptr_t param, ulong interval, int16_t repeat, int16_t priority) {
  // 'repeat' needs to be != 0 for an event to be created
  if (repeat) {
    Event *event = new Event();
    event->callback = callback;
    event->param = param;
    event->interval = interval;
    event->time = interval + millis();
    event->counter = repeat;
    event->priority = priority;
    addEvent(event);
  }
}



/**
 * Add the event to the list sorted by execution time.
 */
void Timer::addEvent(Event *event) {

  // Init
  event->prev = nullptr;
  event->next = nullptr;

  // No events -> add to front
  if (!first) {
    first = event;

  } else if (first->time > event->time) {
    // First one is scheduled later, add new event to front
    event->next = first;
    first->prev = event;
    first = event;

  } else {
    // Add event after temp, so cycle to the next if there is one and the next is executed earlier
    Event *temp = first;
    while (temp->next && temp->next->time <= event->time) {
      temp = temp->next;
    }

    // Set links
    event->next = temp->next;
    temp->next = event;
    event->prev = temp;
    if (event->next) {
      event->next->prev = event;
    }
  }
}

/**
 * Removes the specified event from the linked list.
 */
void Timer::removeEvent(Event *event) {

  if (first == event) {
    first = event->next;
  }
  if (event->prev) {
    event->prev->next = event->next;
  }
  if (event->next) {
    event->next->prev = event->prev;
  }
  event->prev = nullptr;
  event->next = nullptr;
}


void Timer::loop() {

  // If there's nothing to do, return.
  ulong now = millis();
  if (!first || first->time > now) {
    return;
  }

  Event *event = first;
  Event *temp = first->next;

  // Iterate through all events which have their time already passed
  while (temp && temp->time <= now) {
    // Take the event with the highest priority
    if (temp->priority > event->priority) {
      event = temp;
    }
    temp = temp->next;
  }

  removeEvent(event);

  // Store current time and execute callback
  now = millis();
  if (event->timeable) {
    event->timeable->loop();

  } else {
    if (event->param >= 0) {
      (*(event->callback))(event->param);
    } else {
      (*(TimerCallback) (event->callback))();
    }
  }

  // Repeat indefinitely if counter is < 0
  if (event->counter < 0) {
    event->time = now + event->interval;
    addEvent(event);
  } else {

    // Reduce counter and re-add event if above zero
    event->counter--;
    if (event->counter > 0) {
      event->time = now + event->interval;
      addEvent(event);

      // Event is done. Delete it.
    } else {
      delete event;
    }
  }
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_TIMER)
class Timer Timer;
#endif








