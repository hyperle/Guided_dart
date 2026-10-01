#pragma once

// 兼容入口。新代码按配置域包含 detection/config/*.hpp；
// DetectionConfig 仍集中聚合所有运行时策略，保持现有 CLI 和测试接口。
#include "detection/config/detection.hpp"
