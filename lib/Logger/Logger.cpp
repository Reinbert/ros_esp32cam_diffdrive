#include "Logger.h"

Logger_::Logger_() {
  list = new std::list<String*>();
}

Logger_::~Logger_() {
  while (!list->empty()) {
    String *string = list->front();
    list->pop_front();
    delete string;
  }
  delete list;
}

void Logger_::loop() {
  if (list && !list->empty()) {
    String *string = list->front();
    list->pop_front();
    Serial.print(*string);
//    Serial.printf("%s.", string->c_str());
    delete string;
  }
}

size_t Logger_::write(uint8_t c) {
  list->push_back(new String(c));
  return 1;
}

size_t Logger_::write(const uint8_t *buffer, size_t size) {
  list->push_back(new String(reinterpret_cast<const char *>(buffer)));
  return size;
}


#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_LOGGER)
Logger_ Logger;
#endif
