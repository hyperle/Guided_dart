# ============================================================================
# kpu_bench —— AI2D 零拷贝 + KPU 推理的乒乓流水计时器（模型无关）
#
#   VICAP CHN0(RGB_888_PLANAR, ISP 硬件转色彩) -> AI2D(resize+pad, 硬件)
#     -> KPU(nncase interpreter.run，等 KPU 完成中断) -> 计时
#   serial 模式 = 两引擎串行；pipe 模式 = 双缓冲乒乓重叠
#
# 用途：在还没有绿灯 kmodel 前，先用任意 kmodel（如 SDK 自带 best.kmodel）
#       实测「单帧处理能否进 5ms」，为 200fps 目标定模型规格。
#
# 编译: bash scripts/build.sh kpu_bench
# 产物: build/kpu_bench/kpu_bench（链接 nncase 后约 4.7MB）
# 运行: msh> /sdcard/app/kpu_bench --kmodel /sdcard/app/best.kmodel --mode both
# 注意: 与 green_led_ai 互斥（都要独占 VICAP）
# ============================================================================

set(APP_NAME kpu_bench)

add_executable(${APP_NAME} main.cc)

target_compile_options(${APP_NAME} PRIVATE
    -O2 -Wall -Wextra -std=c++17
    -ffunction-sections -fdata-sections)
target_link_options(${APP_NAME} PRIVATE -s)

# nncase 运行时（实测可编译可链接的配方，详见 apps/green_led_ai/README.md）
set(_NN "${K230_SDK_ROOT}/src/rtsmart/libs/nncase")
target_include_directories(${APP_NAME} PRIVATE
    "${_NN}/riscv64"
    "${_NN}/riscv64/nncase/include"
    "${_NN}/riscv64/nncase/include/nncase/runtime"
    "${_NN}/riscv64/rvvlib/include")
target_link_directories(${APP_NAME} PRIVATE
    "${_NN}/riscv64/nncase/lib"
    "${_NN}/riscv64/rvvlib"
    "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/lib")

target_link_libraries(${APP_NAME} PRIVATE
    k230_rtos_app
    -Wl,--start-group Nncase.Runtime.Native nncase.rt_modules.k230 functional_k230 rvv
    sys vicap vb isp sensor gsdma -Wl,--end-group pthread m)

set_target_properties(${APP_NAME} PROPERTIES
    OUTPUT_NAME ${APP_NAME}
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${APP_NAME}")
