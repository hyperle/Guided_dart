#pragma once

// 兼容入口：新代码按 domain/ 和 math/ 的职责选择头文件；旧调用方继续包含本文件。
#include "detection/domain/geometry.hpp"
#include "detection/domain/observations.hpp"
#include "detection/domain/tracking.hpp"
#include "detection/math/shape_metrics.hpp"
