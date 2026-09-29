#pragma once
#include "hardware/attitude_resolve.hpp"
#include "hardware/device/IIM42622.hpp"

// 在配置文件中统一更换具体实现，无需修改任何业务代码
using IMUDriver = dart::hardware::device::IIM42622Driver;
using AttitudeSolver = dart::hardware::AttitudeSolver<IMUDriver>;