#include <napi.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "../lib/helper_3dmath.h"
#include "../lib/mpu/inv_mpu.h"
#include "../lib/mpu/inv_mpu_dmp_motion_driver.h"

using namespace Napi;

uint8_t GetGravity(VectorFloat *v, Quaternion *q);
uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

Array loadInArray(Env env, float *values);

Boolean startMpu(const CallbackInfo& info);
void readMpu(const CallbackInfo& info);
void setSampleFreq(const Napi::CallbackInfo& info);
void setDlpf(const CallbackInfo& info);

Object Init(Env env, Object exports);
