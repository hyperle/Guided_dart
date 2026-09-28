# ============================================================================
# src —— 导引主程序（采集 → 二值化 → 识别/跟踪 → 录像）
#
#   采集: SensorSource  —— VICAP OFFLINE 单通道 CHN0，1280x720@90 缩放到 640x360
#   二值化: GraphicsUtils::binarize —— RVV 向量（写进池帧，录像/取证/识别共用同一张二值图）
#   识别: DetectionPipeline（dart::detection）—— 启动态全图 RVV 粗筛 + 3 帧滑窗确认；
#         跟踪态动态 ROI + 连通域测量 + 带尺度预测的卡尔曼滤波（详见 DETECTION_DESIGN.md）
#   录像: Recorder —— 池帧零拷贝送 VENC 编黑白 H.264，左上角烧入关键数据
#
# 编译: bash scripts/build.sh self_guiding_dart
# ============================================================================

set(APP_NAME self_guiding_dart)

add_executable(${APP_NAME}
    main.cpp
    core/log.cpp
    core/mpp_map.cpp
    core/mmz.cpp
    evidence/evidence_writer.cpp
    capture/sensor_source.cpp
    vision/frame_pool.cpp
    recording/recorder.cpp
    detection/light.cpp
    detection/blob_measure.cpp
    detection/armor.cpp
    detection/kalman.cpp
    detection/roi_prediction.cpp
    detection/tracker.cpp
    detection/pipeline.cpp
    detection/armor/armor_geometry.cpp
    detection/armor/armor_scale.cpp
    detection/armor/armor_rules.cpp
    detection/armor/armor_windows.cpp
    detection/armor/armor_detector.cpp
    control/flight_state_machine.cpp
    control/visual_outer_loop.cpp
    control/attitude_inner_loop.cpp
    control/control_allocator.cpp
    control/flight_controller.cpp)

target_include_directories(${APP_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")

set_target_properties(${APP_NAME} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    OUTPUT_NAME ${APP_NAME}
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${APP_NAME}")

# -O2 覆盖工具链默认的 -Os：二值化内层循环吃 RVV 的 LMUL 宽度
target_compile_options(${APP_NAME} PRIVATE -O2 -Wall -Wextra -ffunction-sections -fdata-sections)
target_link_options(${APP_NAME} PRIVATE -s)

target_link_directories(${APP_NAME} PRIVATE
    "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/lib")
target_link_libraries(${APP_NAME} PRIVATE
    k230_rtos_app
    # atomic：RISC-V 没有子字长原子指令，1 字节 atomic 走 libatomic
    -Wl,--start-group vicap vb sys sensor isp venc pm atomic -Wl,--end-group
    pthread m)
