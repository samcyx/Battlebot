#include <BattleBotDetection.h>

BattleBotDetection::BattleBotDetection() {//constructor
}

void BattleBotDetection::begin() {
    delay(3000);
    Serial.begin(115200);
    Serial.println("__EDGE IMPULSE FOMO (NO-PSRAM)__");

 // Configure the camera pins for Freenove ESP32-S3 WROOM
    camera.pinout.freenove_s3();


  
     camera.brownout.disable();
    // NON-PSRAM FOMO only works on 96x96 (yolo) RGB565 images
    camera.resolution.yolo();
    camera.pixformat.rgb565();
    delay(3000);
    // init camera
    while (!camera.begin().isOk())
    {
      Serial.println(camera.exception.toString());
      delay(3000);
    }
}

std::tuple<int, int> BattleBotDetection::detectX() {
    // Code to run the detection (similar to your original code)
       if (!camera.capture().isOk()) {
        Serial.println(camera.exception.toString());
        returnmake_tuple(0,0);
    }

    // run FOMO
    if (!fomo.run().isOk()) {
      Serial.println(fomo.exception.toString());
      returnmake_tuple(0,0);
    }

    // how many objects were found?
    Serial.printf(
      "Found %d object(s) in %dms\n", 
      fomo.count(),
      fomo.benchmark.millis()
    );

    // if no object is detected, return
    if (!fomo.foundAnyObject())
      return make_tuple(0,0);

    // if you expect to find a single object, use fomo.first
    /*
    Serial.printf(
      "Found %s at (x = %d, y = %d) (size %d x %d). "
      "Proba is %.2f\n",
      fomo.first.label,
      fomo.first.x,
      fomo.first.y,
      fomo.first.width,
      fomo.first.height,
      fomo.first.proba
    );
    */
    int x = int(fomo.first.x())
    int probability = int(fomo.first.proba())
    return make_tuple( x,probability );
}

void BattleBotDetection::printResults() {
    // Print detection results to the serial monitor
    fomo.forEach([](int i, bbox_t bbox) {
        Serial.printf(
            "#%d) Found %s at (x = %d, y = %d) (size %d x %d). "
            "Proba is %.2f\n",
            i + 1,
            bbox.label,
            bbox.x,
            bbox.y,
            bbox.width,
            bbox.height,
            bbox.proba
        );
    });
}
