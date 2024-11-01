#ifndef Battlebots_inferencing.h
#define Battlebots_inferencing.h
#include <tuple>   // Include the tuple header

#include "Battlebots_inferencing.h"
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/edgeimpulse/fomo.h>
using eloq::camera;
using eloq::ei::fomo;
class BattleBotDetection {
public:


  BattleBotDetection();  // Constructor
  std::tuple<int, int> detectX(); // Return a tuple with detection count and results

  void begin();          // Initialization function
  void printResults();   // Function to print results

};

#endif
