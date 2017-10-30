#include <iostream>

class task{

private:

  bool isActive;
  cv::Point currentTargetPosition; // target position when the frame is not empty
  cv::Point lastTargetPosition; // target last position when the frame was not empty
  float contourSize;
  float distance;

public:

  task(); // default contructor
  ~task(); // default destructor
  bool getState(); // is the task is active i.e. if it looking for any targets
  void isTargetDetected(); // if the target is detected
  void emptyFrameHandler(); // to tell where the hell was target in the previous frame
  void getPosition(); // if the target is in the current frame
  void getDistance();
  void getContourSize();

};
