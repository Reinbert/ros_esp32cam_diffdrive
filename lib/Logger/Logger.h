#ifndef LOGGER_H
#define LOGGER_H

#include <Print.h>
#include <list>
#include <Timer.h>

class Logger_ : public Timeable, public Print {
public:
  Logger_();
  virtual ~Logger_();
  void loop() override;

  size_t write(uint8_t) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  using Print::write;

protected:
  std::list<String*> *list;
protected:

};

extern Logger_ Logger;

#endif