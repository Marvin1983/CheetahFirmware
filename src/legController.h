#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

class legController {
private:
  class motorController {

  private:
    int canID;

  public:
    float posEst, speedEst, touqueEst; // estimate position, speed, touque
    float kp, kd;
    float posRef, speedEstRef, touqueRef; // reference position, speed, touque

    motorController(int canID, float initPos);
    ~motorController();

  } roll, hip, knee;

public:
  legController();
  ~legController();
};

legController::motorController::motorController(int canID, float initPos) {}

legController::motorController::~motorController() {}

legController::legController() {}

legController::~legController() {}

#endif
