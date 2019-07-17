# InvenSense MPU Motion Driver

Asynchronous NodeJS module for accessing the DMP and sensor data from the InvenSense motion sensors. 
Based on the official MotionDriver v5. Adapted from `rpicopter/MotionSensorExample`.

## Adaption for other MPU boards
If necessary, adjust the defines in `binding.gyp` to match your board.

## Testing
A simple program would look like this:
```
const driver = require('js-inv-mpu');
driver.setSampleFreq(200);  // sampling frequency - default value
driver.setDlpf(20);         // digital low pass filter - default value
driver.measure(console.log);
```