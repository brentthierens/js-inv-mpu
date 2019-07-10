#include <napi.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <pthread.h>

#include "../lib/helper_3dmath.h"
#include "../lib/mpu/inv_mpu.h"
#include "../lib/mpu/inv_mpu_dmp_motion_driver.h"

using namespace Napi;

uint8_t GetGravity(VectorFloat *v, Quaternion *q);
uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

void* readMpu(void* args);
Array loadInArray(Env env, float *values);

Boolean startMpu(const CallbackInfo& info);
void setSampleFreq(const Napi::CallbackInfo& info);
Array getYpr(const CallbackInfo& info);
Array getGyro(const CallbackInfo& info);
Array getAccel(const CallbackInfo& info);
Array getCompas(const CallbackInfo& info);
Number getTemp(const CallbackInfo& info);
Object Init(Env env, Object exports);
