# ============================================================================
# green_led_ai —— 绿灯识别 + H.264 录像，单进程双子系统（VICAP 唯一持有者）
#
#   采集: VICAP OFFLINE，CHN0=视觉通道，CHN1=录像通道（互不共享帧）
#   视觉: 后端可插拔
#           color —— RVV 色彩变换 + CIELAB 查表阈值 + 连通域（无模型，默认）
#           kpu   —— AI2D 硬件预处理(零拷贝绑物理地址) + KPU 乒乓异步推理
#                    + YOLOv8 解码/NMS（自研，有宿主机单元测试；SDK 的 librvv
#                      只提供数学核，nms.h 里的 nms() 库里并没有实现）
#   录像: VENC 直接吃 VICAP 帧物理地址（零拷贝），25fps 抽样，独立启停
#
# 两个产物（同一套源码，只有 KPU 后端不同）：
#   green_led_ai       —— 精简版，不链 nncase（约 0.9MB），只有 color 后端
#   green_led_ai_kpu   —— 链接 nncase + librvv（约 5MB），多出 --detector kpu
#
# 编译: bash scripts/build.sh green_led_ai
#       bash scripts/build.sh green_led_ai_kpu
# 部署: bash scripts/deploy-sdcard.sh <SD卡挂载点>
# ============================================================================

set(APP_NAME green_led_ai)

set(COMMON_SRC
    main.c
    app.c
    control.c
    detect_color.c
    postproc.c
    rec_dir.c
    record.c
    vicap_src.c
    vision.c)

set(_NN "${K230_SDK_ROOT}/src/rtsmart/libs/nncase")

# -O2 覆盖工具链默认的 -Os：检测器内层循环吃 RVV/展开，-Os 会限制它
set(COMMON_OPT -O2 -Wall -Wextra -ffunction-sections -fdata-sections)

# ---------------------------------------------------------------------------
# 1) 精简版：color 后端（无 nncase 依赖）
# ---------------------------------------------------------------------------
add_executable(${APP_NAME} ${COMMON_SRC})
target_compile_options(${APP_NAME} PRIVATE ${COMMON_OPT})
target_link_options(${APP_NAME} PRIVATE -s)
target_link_directories(${APP_NAME} PRIVATE
    "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/lib")
target_link_libraries(${APP_NAME} PRIVATE
    k230_rtos_app
    -Wl,--start-group vicap vb sys sensor isp venc pm
    -Wl,--end-group pthread m)

# ---------------------------------------------------------------------------
# 2) KPU 版：color + kpu 后端（链接 nncase 运行时与 librvv）
# ---------------------------------------------------------------------------
add_executable(${APP_NAME}_kpu ${COMMON_SRC} detect_kpu.cpp)
target_compile_options(${APP_NAME}_kpu PRIVATE ${COMMON_OPT})
target_compile_definitions(${APP_NAME}_kpu PRIVATE GREEN_LED_WITH_KPU=1)
target_link_options(${APP_NAME}_kpu PRIVATE -s)
target_include_directories(${APP_NAME}_kpu PRIVATE
    "${_NN}/riscv64"
    "${_NN}/riscv64/nncase/include"
    "${_NN}/riscv64/nncase/include/nncase/runtime"
    "${_NN}/riscv64/rvvlib/include")
target_link_directories(${APP_NAME}_kpu PRIVATE
    "${_NN}/riscv64/nncase/lib"
    "${_NN}/riscv64/rvvlib"
    "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/lib")
target_link_libraries(${APP_NAME}_kpu PRIVATE
    k230_rtos_app
    -Wl,--start-group vicap vb sys sensor isp venc pm
    Nncase.Runtime.Native nncase.rt_modules.k230 functional_k230
    -Wl,--end-group pthread m)

foreach(_t ${APP_NAME} ${APP_NAME}_kpu)
    set_target_properties(${_t} PROPERTIES
        OUTPUT_NAME ${_t}
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${_t}")
endforeach()
