{
    "targets": [{
        "target_name": "js-inv-mpu",
        "cflags_cc!": [ "-fno-exceptions", "-pthread" ],
        "sources": [
            "src/main.cc",
            "lib/helper_3dmath.h",
            "<!@(node -p \"require('fs').readdirSync('./lib/mpu').map(f=>'lib/mpu/'+f).join(' ')\")",
            "<!@(node -p \"require('fs').readdirSync('./lib/i2c').map(f=>'lib/i2c/'+f).join(' ')\")"
        ],
        'include_dirs': [
            "<!@(node -p \"require('node-addon-api').include\")"
        ],
        'dependencies': [
            "<!(node -p \"require('node-addon-api').gyp\")"
        ],
        'defines': [ 'NAPI_DISABLE_CPP_EXCEPTIONS', 'MPU6050' ]
    }]
}