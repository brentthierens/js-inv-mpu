#include "main.h"

int dmpReady;
int16_t sensors;
uint16_t rate;
uint16_t dlpf;

float ypr[3];
float gyro[3];
float accel[3];
float compass[3];
float temp;

uint8_t GetGravity(VectorFloat *v, Quaternion *q)
{
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity)
{
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}

Boolean startMpu(const CallbackInfo& info)
{
    Env env = info.Env();

    dmpReady=1;

	printf("Initializing MPU...\n");
	if (mpu_init(NULL) != 0) {
		printf("MPU init failed!\n");
		return Boolean::New(env, false);
	}

	printf("Setting MPU sensors...\n");
	if (mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0) {
		printf("Failed to set sensors!\n");
		return Boolean::New(env, false);
	}

	printf("Setting GYRO sensitivity...\n");
	if (mpu_set_gyro_fsr(2000)!=0) {
		printf("Failed to set gyro sensitivity!\n");
		return Boolean::New(env, false);
	}

	printf("Setting ACCEL sensitivity...\n");
	if (mpu_set_accel_fsr(2)!=0) {
		printf("Failed to set accel sensitivity!\n");
		return Boolean::New(env, false);
	}

	// verify connection
    uint8_t devStatus;
	printf("Powering up MPU...\n");
	mpu_get_power_state(&devStatus);
	printf(devStatus ? "MPU connection successful\n" : "MPU connection failed %u\n",devStatus);

	//fifo config
	printf("Setting MPU fifo...\n");
	if (mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0) {
		printf("Failed to initialize MPU fifo!\n");
		return Boolean::New(env, false);
	}

	// load and configure the DMP
	printf("Loading DMP firmware...\n");
	if (dmp_load_motion_driver_firmware()!=0) {
		printf("Failed to enable DMP!\n");
		return Boolean::New(env, false);
	}

	printf("Activating DMP...\n");
	if (mpu_set_dmp_state(1)!=0) {
		printf("Failed to enable DMP!\n");
		return Boolean::New(env, false);
	}

	printf("Configuring DMP...\n");
	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL)!=0) {
		printf("Failed to enable DMP features!\n");
		return Boolean::New(env, false);
	}

	printf("Setting DMP fifo rate...\n");
	if (dmp_set_fifo_rate(rate)!=0) {
		printf("Failed to set dmp fifo rate!\n");
		return Boolean::New(env, false);
	}
	printf("Resetting fifo queue...\n");
	if (mpu_reset_fifo()!=0) {
		printf("Failed to reset fifo!\n");
		return Boolean::New(env, false);
	}

    if (dlpf && mpu_set_lpf(dlpf) != 0) {
        printf("Failed to set low pass filter!\n");
        return Boolean::New(env, false);
    }

	printf("Done, starting measurements.\n");
    pthread_t thread;
    pthread_create(&thread, NULL, readMpu, NULL);

    return Boolean::New(env, true);
}

void* readMpu(void* args)
{
    if (!dmpReady) {
        printf("Error: DMP not ready!!\n");
        return NULL;
    }

    uint8_t fifoCount;
    int16_t g[3];
    int16_t a[3];
    int16_t c[3];
    int32_t _q[4];
    int32_t t;
    Quaternion q;
    VectorFloat gravity;

    while(true)
    {
        while (dmp_read_fifo(g, a, _q, &sensors, &fifoCount) != 0);
        q = _q;

        GetGravity(&gravity, &q);
        GetYawPitchRoll(ypr, &q, &gravity);

        mpu_get_temperature(&t);
        temp=(float)t/65536L;

        mpu_get_compass_reg(c);

        //scaling for degrees output
        for (int i=0;i<3;i++) ypr[i]*=180/M_PI;
        //unwrap yaw when it reaches 180
        ypr[0] = (ypr[0] < -180 ? ypr[0]+360 : (ypr[0] > 180 ? ypr[0] - 360 : ypr[0]));
        //change sign of Pitch, MPU is attached upside down
        ypr[1]*=-1.0;

        //0=gyroX, 1=gyroY, 2=gyroZ
        //swapped to match Yaw,Pitch,Roll
        for (int i=0;i<3;i++)
        {
            gyro[i]    = (float)(g[3-i-1])/131.0;
            accel[i]   = (float)(a[3-i-1])/16384.0;
            compass[i] = (float)(c[3-i-1]);
        }

        usleep(1e6/rate);
    }

    return NULL;
}

void setSampleFreq(const CallbackInfo& info)
{
    Number freq = info[0].As<Number>();
    rate = freq.Int32Value();
}

void setDlpf(const CallbackInfo& info)
{
    Number freq = info[0].As<Number>();
    dlpf = freq.Int32Value();
}

Array loadInArray(Env env, float *values)
{
    Array result = Array::New(env, 3);
    for(uint8_t iterator = 0; iterator < 3; iterator++)
        result[iterator] = values[iterator];
    return result;
}

Array getYpr(const CallbackInfo& info)
{
    return loadInArray(info.Env(), ypr);
}

Array getGyro(const CallbackInfo& info)
{
    return loadInArray(info.Env(), gyro);
}

Array getAccel(const CallbackInfo& info)
{
    return loadInArray(info.Env(), accel);
}

Array getCompass(const CallbackInfo& info)
{
    return loadInArray(info.Env(), compass);
}

Number getTemp(const CallbackInfo& info)
{
    return Number::New(info.Env(), temp);
}

Object Init(Env env, Object exports)
{
    exports.Set("startMpu",        Function::New(env, startMpu));
    exports.Set("setSampleFreq",   Function::New(env, setSampleFreq));
    exports.Set("setDlpf",         Function::New(env, setDlpf));
    exports.Set("getYpr",          Function::New(env, getYpr));
    exports.Set("getGyro",         Function::New(env, getGyro));
    exports.Set("getAccel",        Function::New(env, getAccel));
    exports.Set("getCompass",      Function::New(env, getCompass));
    exports.Set("getTemp",         Function::New(env, getTemp));

    return exports;
}

NODE_API_MODULE(jsInvMpu, Init)