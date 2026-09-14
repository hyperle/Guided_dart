/*
 * detect_kpu —— 异步 KPU 检测后端：AI2D 零拷贝预处理 + KPU 乒乓流水 + 后处理
 *
 * 线程模型（三条流水重叠，任何一级都不会长时间占着 MPP 锁）：
 *
 *   vision 处理线程 (C 侧)
 *     └ submit(frame, view, seq)  —— 只投递，不阻塞；帧所有权转交本后端
 *          ↓  单槽/双缓冲调度
 *   pre 线程 (这里)
 *     ├ 用 VICAP 帧的**物理地址**直接建 AI2D 输入 tensor（hrt::create，
 *     │  copy=false + physical_address：零拷贝，不做任何 memcpy）
 *     ├ ai2d_builder::invoke()  —— 硬件 resize+pad，内部 poll 等 AI2D 中断
 *     └ 归还 VICAP 帧（AI2D 已读完）→ 该缓冲标记 READY
 *          ↓
 *   kpu 线程 (这里)
 *     ├ interpreter::run()      —— nncase 内部 poll 等 **KPU 完成中断**（不是轮询）
 *     ├ 读输出 tensor → YOLOv8 解码 → NMS（默认自研；可选 SDK 自带 librvv 的 nms()）
 *     └ 推一条结果进结果环（供 vision 线程 collect）
 *
 * 双缓冲乒乓的意义：pre 线程做第 N+1 帧的 AI2D 时，kpu 线程可以同时在跑第 N 帧
 * 的 KPU —— AI2D 与 KPU 是两个独立硬件引擎，重叠后单帧吞吐 ≈ max(两者) 而不是和。
 *
 * 与 nncase/KPU 相关的 API 用法照抄官方示例
 *   src/rtsmart/examples/ai/usage_kpu/yolov8_run_camera/main.cc
 * （hrt::create 绑物理地址、ai2d_builder 的 letterbox 参数、interp.run()），
 * 只是把它拆成了「异步 + 乒乓 + 可插拔后处理」。
 */
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <string>
#include <vector>

#include <nncase/functional/ai2d/ai2d_builder.h>
#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <nncase/runtime/runtime_tensor.h>

extern "C" {
#include "app.h"
#include "detect.h"
#include "postproc.h"
#include "vicap_src.h"
}

using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::F::k230;

/* ============================ 配置（来自 g_cfg） ============================ */

#define KPU_SLOTS      2      /* 乒乓缓冲数 */
#define RESULT_RING    8      /* 结果环深度 */

enum { ST_FREE = 0, ST_SCHED = 1, ST_PREP = 2, ST_READY = 3, ST_KPU = 4 };

struct slot_t {
    runtime_tensor              in;      /* AI2D 输出 = KPU 输入（同一份内存） */
    std::vector<runtime_tensor> out;     /* KPU 输出 tensor（每个输出一个） */
    int                         state;

    /* 该 slot 上正在处理的任务 */
    k_video_frame_info frame;
    frame_view_t       view;
    uint64_t           seq;
    uint64_t           submit_us;
    uint64_t           ai2d_us;
};

static interpreter              g_interp;
static ai2d_builder            *g_builder = nullptr;
static slot_t                   g_slot[KPU_SLOTS];
static dims_t                   g_src_shape;    /* AI2D 输入：{1,3,src_h,src_w} */
static dims_t                   g_model_shape;  /* AI2D 输出 = 模型输入 */
static uint32_t                 g_src_bytes;
static int                      g_src_w, g_src_h;

static pthread_t                g_pre_tid, g_kpu_tid;
static int                      g_pre_started, g_kpu_started;
static pthread_mutex_t          g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t           g_cond = PTHREAD_COND_INITIALIZER;
static volatile int             g_run;

/* 结果环 */
static detect_out_t             g_res[RESULT_RING];
static int                      g_res_head, g_res_tail, g_res_cnt;
static pthread_mutex_t          g_res_lock = PTHREAD_MUTEX_INITIALIZER;

/* 统计 */
static uint64_t                 g_n_submit, g_n_busy, g_n_ai2d_err, g_n_kpu_err, g_n_done, g_n_dropped;
static uint64_t                 g_t_ai2d_sum, g_t_ai2d_max;
static uint64_t                 g_t_kpu_sum, g_t_kpu_max;
static uint64_t                 g_t_post_sum, g_t_post_max;
static uint64_t                 g_t_lat_sum, g_t_lat_max;
static uint64_t                 g_stat_start_us;
static float                    g_conf = 0.35f, g_iou = 0.45f;
static int                      g_class_filter = -1;
static int                      g_layout_cmajor = 1;   /* 1 = out[c*n_box+i] */
static int                      g_nc, g_n_box;
static int                      g_out_is_float = 1;

/* ============================ 小工具 ============================ */

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

static void klog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void klog(const char *fmt, ...)
{
    char buf[640];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
        return;
    /* app_log 会加前缀并落盘；这里只是薄封装，保持 kpu: 前缀便于检索 */
    app_log(APP_NAME_STR ": kpu: %s", buf);
}

/* 结果环：满则丢最旧（后处理不该把采集/推理卡住） */
static void result_push(const detect_out_t *d)
{
    pthread_mutex_lock(&g_res_lock);
    if (g_res_cnt == RESULT_RING) {
        g_res_tail = (g_res_tail + 1) % RESULT_RING;
        g_res_cnt--;
        g_n_dropped++;
    }
    g_res[g_res_head] = *d;
    g_res_head = (g_res_head + 1) % RESULT_RING;
    g_res_cnt++;
    pthread_mutex_unlock(&g_res_lock);
}

/* ============================ 后处理 ============================ */

static int post_process(slot_t *s, detect_out_t *d)
{
    memset(d, 0, sizeof(*d));
    d->cx = -1;
    d->cy = -1;
    d->cls = -1;
    d->seq = s->seq;

    uint64_t t0 = now_us();

    /*
     * 取模型第 0 个输出的可读缓冲区。
     * 这里刻意与官方示例（usage_kpu/yolov8_run_camera）写法一致：链式取到
     * buffer_slice 再拿 data()，不长期持有 mapped_buffer（该类型不可拷贝，
     * 官方示例也是这么用的，说明这条路径的映射在 slice 生命周期内一直有效）。
     */
    auto ot = g_interp.output_tensor(0);
    if (!ot.is_ok()) {
        klog("output_tensor(0) failed\n");
        return -1;
    }
    auto slice = ot.unwrap().impl()->to_host().unwrap()->buffer().as_host().unwrap()
                     .map(map_access_::map_read).unwrap().buffer();
    const float *pf = reinterpret_cast<const float *>(slice.data());

    det_box_t boxes[256];
    int n = 0;

    if (!g_out_is_float) {
        klog("输出不是 float32（量化模型暂不支持），跳过本帧\n");
        return -1;
    }

    n = yolov8_decode(pf, g_n_box, g_nc + 4, g_layout_cmajor, g_nc,
                      g_conf, (float)g_model_shape[3] / (float)g_src_w,
                      boxes, (int)(sizeof(boxes) / sizeof(boxes[0])));

    int n_before_nms = n;

    /*
     * NMS：用本仓库自研的 nms_boxes()（apps/green_led_ai/postproc.c，有宿主机
     * 单元测试）。**没有**用 SDK 的 librvv：实测 librvv.a 里只有数学核
     * （softmax_vec/expf_vec/sigmoid_vec/sgemm_vec 等 32 个符号），
     * rvvlib/include/nms.h 里声明的 nms()/nms_e() 在库里没有实现，
     * 官方 yolov8 示例也是自己写的 NMS。
     */
    n = nms_boxes(boxes, n, g_iou);

    int bi = pick_best(boxes, n, g_class_filter);
    if (bi >= 0) {
        float cx = 0.5f * (boxes[bi].x1 + boxes[bi].x2);
        float cy = 0.5f * (boxes[bi].y1 + boxes[bi].y2);
        d->cx = (int32_t)cx;
        d->cy = (int32_t)cy;
        d->score = boxes[bi].score;
        d->cls = boxes[bi].cls;
        d->px = (uint32_t)((boxes[bi].x2 - boxes[bi].x1) * (boxes[bi].y2 - boxes[bi].y1));
    }
    d->blobs = (uint32_t)n;
    d->roi_w = (uint32_t)g_src_w;
    d->roi_h = (uint32_t)g_src_h;

    uint64_t t1 = now_us();
    d->t_us = (uint32_t)(t1 - t0);        /* 后处理耗时 */
    d->y_mean = (uint32_t)n_before_nms;   /* 借用该字段回报「NMS 前候选数」 */

    /* 端到端延迟：从 submit（帧刚 dump 出来）到结果产生 */
    uint64_t lat = t1 - s->submit_us;
    d->lat_us = (uint32_t)lat;
    pthread_mutex_lock(&g_lock);
    g_t_post_sum += d->t_us;
    if (d->t_us > g_t_post_max)
        g_t_post_max = d->t_us;
    g_t_lat_sum += lat;
    if (lat > g_t_lat_max)
        g_t_lat_max = lat;
    pthread_mutex_unlock(&g_lock);

    return 0;
}

/* ============================ 线程 ============================ */

static int slot_pick(int want_state)
{
    int idx = -1;
    pthread_mutex_lock(&g_lock);
    for (;;) {
        for (int i = 0; i < KPU_SLOTS; ++i) {
            if (g_slot[i].state == want_state) {
                idx = i;
                break;
            }
        }
        if (idx >= 0 || !g_run)
            break;
        pthread_cond_wait(&g_cond, &g_lock);
    }
    pthread_mutex_unlock(&g_lock);
    return idx;
}

static void slot_state_set(int idx, int st)
{
    pthread_mutex_lock(&g_lock);
    g_slot[idx].state = st;
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);
}

/* pre 线程：AI2D 硬件预处理（零拷贝绑定 VICAP 帧），完成后归还帧 */
static void *pre_thread(void *arg)
{
    (void)arg;
    for (;;) {
        int idx = slot_pick(ST_SCHED);
        if (idx < 0)
            break;

        slot_t *s = &g_slot[idx];
        pthread_mutex_lock(&g_lock);
        s->state = ST_PREP;
        pthread_mutex_unlock(&g_lock);

        uint64_t t0 = now_us();
        int ok = -1;

        uint64_t phys0 = s->frame.v_frame.phys_addr[0];
        uint32_t w = s->frame.v_frame.width;
        uint32_t h = s->frame.v_frame.height;
        uint32_t stride = s->frame.v_frame.stride[0] ? s->frame.v_frame.stride[0] : w;
        int contiguous = (stride == w) &&
                         (s->frame.v_frame.phys_addr[1] == phys0 + (uint64_t)w * h) &&
                         (s->frame.v_frame.phys_addr[2] == phys0 + 2ull * w * h);

        if (contiguous && w == (uint32_t)g_src_w && h == (uint32_t)g_src_h && s->view.y) {
            /* 零拷贝：AI2D 输入 tensor 直接盖在 VICAP 帧的物理地址上 */
            dims_t shape{ 1, (size_t)3, (size_t)h, (size_t)w };
            auto res = host_runtime_tensor::create(
                typecode_t::dt_uint8, shape,
                { (gsl::byte *)s->view.y, (size_t)g_src_bytes },
                false, hrt::pool_shared, (uintptr_t)phys0);
            if (res.is_ok()) {
                runtime_tensor in = res.unwrap();
                if (g_cfg.kpu_sync)
                    (void)hrt::sync(in, sync_op_t::sync_write_back, true);
                auto r = g_builder->invoke(in, s->in);   /* 内部等 AI2D 中断 */
                if (r.is_ok())
                    ok = 0;
                else
                    klog("ai2d invoke failed\n");
            } else {
                klog("hrt::create over vicap frame failed\n");
            }
        } else if (!contiguous) {
            klog("帧平面不连续（stride=%u phys=%llx/%llx/%llx），无法零拷贝绑定\n",
                 stride, (unsigned long long)s->frame.v_frame.phys_addr[0],
                 (unsigned long long)s->frame.v_frame.phys_addr[1],
                 (unsigned long long)s->frame.v_frame.phys_addr[2]);
        } else {
            klog("帧尺寸不符 src=%dx%d\n", g_src_w, g_src_h);
        }

        uint64_t t1 = now_us();

        /* AI2D 已读完帧（invoke 返回即硬件完成）-> 归还 VICAP 帧 */
        vicap_release(VICAP_CHN_VISION, &s->frame);
        s->view.y = NULL;

        pthread_mutex_lock(&g_lock);
        s->ai2d_us = t1 - t0;
        g_t_ai2d_sum += (t1 - t0);
        if (t1 - t0 > g_t_ai2d_max)
            g_t_ai2d_max = t1 - t0;
        if (ok != 0)
            g_n_ai2d_err++;
        s->state = (ok == 0) ? ST_READY : ST_FREE;
        pthread_cond_broadcast(&g_cond);
        pthread_mutex_unlock(&g_lock);
    }
    return nullptr;
}

/* kpu 线程：KPU 推理（等完成中断）+ 后处理 */
static void *kpu_thread(void *arg)
{
    (void)arg;
    for (;;) {
        int idx = slot_pick(ST_READY);
        if (idx < 0)
            break;

        slot_t *s = &g_slot[idx];
        pthread_mutex_lock(&g_lock);
        s->state = ST_KPU;
        pthread_mutex_unlock(&g_lock);

        int ok = -1;
        auto iset = g_interp.input_tensor(0, s->in);
        if (iset.is_ok()) {
            ok = 0;
            for (size_t i = 0; i < s->out.size(); ++i) {
                auto os = g_interp.output_tensor(i, s->out[i]);
                if (!os.is_ok()) {
                    klog("set output tensor %u failed\n", (unsigned)i);
                    ok = -1;
                    break;
                }
            }
        } else {
            klog("set input tensor failed\n");
        }

        if (ok == 0) {
            uint64_t t0 = now_us();
            auto r = g_interp.run();      /* 内部 poll 等 KPU 完成中断 */
            uint64_t t1 = now_us();
            if (r.is_ok()) {
                pthread_mutex_lock(&g_lock);
                g_t_kpu_sum += (t1 - t0);
                if (t1 - t0 > g_t_kpu_max)
                    g_t_kpu_max = t1 - t0;
                pthread_mutex_unlock(&g_lock);

                detect_out_t d;
                if (post_process(s, &d) == 0) {
                    result_push(&d);
                    pthread_mutex_lock(&g_lock);
                    g_n_done++;
                    pthread_mutex_unlock(&g_lock);
                }
            } else {
                klog("interp.run failed\n");
                pthread_mutex_lock(&g_lock);
                g_n_kpu_err++;
                pthread_mutex_unlock(&g_lock);
            }
        } else {
            pthread_mutex_lock(&g_lock);
            g_n_kpu_err++;
            pthread_mutex_unlock(&g_lock);
        }

        slot_state_set(idx, ST_FREE);
    }
    return nullptr;
}

/* ============================ detector 接口 ============================ */

static int kpu_init(void)
{
    if (!g_cfg.kmodel || !g_cfg.kmodel[0]) {
        klog("缺少 --kmodel，无法启用 kpu 后端\n");
        return -1;
    }

    std::ifstream ifs(g_cfg.kmodel, std::ios::binary);
    if (!ifs.good()) {
        klog("打不开 kmodel: %s\n", g_cfg.kmodel);
        return -1;
    }
    auto lr = g_interp.load_model(ifs);
    if (!lr.is_ok()) {
        klog("load_model 失败: %s\n", g_cfg.kmodel);
        return -1;
    }

    if (g_interp.outputs_size() < 1) {
        klog("模型没有输出\n");
        return -1;
    }

    /* AI2D 预处理要求输入是 uint8（NCHW/平面），输出要能按 float 读 */
    int in_dt = (int)g_interp.input_desc(0).datatype;
    int out_dt = (int)g_interp.output_desc(0).datatype;
    klog("模型 dtype: input=%d output=%d（期望 1=uint8 输入 / 9=float32 输出）\n",
         in_dt, out_dt);
    if (in_dt != (int)dt_uint8) {
        klog("输入不是 uint8（量化模型一般输入 uint8）；本后端暂不支持，退出\n");
        return -1;
    }
    if (out_dt != (int)dt_float32) {
        klog("输出不是 float32（量化输出需要 dequant，暂不支持），退出\n");
        return -1;
    }

    g_model_shape = g_interp.input_shape(0);
    if (g_model_shape.size() != 4) {
        klog("只支持 4 维输入模型（当前 %u 维）\n", (unsigned)g_model_shape.size());
        return -1;
    }
    if (g_interp.inputs_size() != 1) {
        klog("警告：模型有 %u 个输入，只用第 0 个\n", (unsigned)g_interp.inputs_size());
    }

    /* 输出布局判定：YOLOv8 单输出 (1, 4+nc, n_box) 或 (1, n_box, 4+nc) */
    auto oshape = g_interp.output_shape(0);
    if (oshape.size() != 3 && oshape.size() != 4) {
        klog("只支持 3/4 维输出（当前 %u 维）\n", (unsigned)oshape.size());
        return -1;
    }
    int d1 = (int)oshape[oshape.size() - 2];
    int d2 = (int)oshape[oshape.size() - 1];
    if (g_cfg.kpu_nc > 0) {
        g_nc = g_cfg.kpu_nc;
        g_n_box = (d1 == g_nc + 4) ? d2 : d1;
        g_layout_cmajor = (d1 == g_nc + 4) ? 1 : 0;
    } else {
        /* 自动：通道数必然远小于候选数 */
        if (d1 < d2) {
            g_nc = d1 - 4;
            g_n_box = d2;
            g_layout_cmajor = 1;
        } else {
            g_nc = d2 - 4;
            g_n_box = d1;
            g_layout_cmajor = 0;
        }
    }
    if (g_nc <= 0 || g_n_box <= 0) {
        klog("输出形状无法解析：%dx%d\n", d1, d2);
        return -1;
    }
    g_out_is_float = true;   /* 已在上方校验过 dt_float32 */

    g_src_w = (int)vicap_chn_width(VICAP_CHN_VISION);
    g_src_h = (int)vicap_chn_height(VICAP_CHN_VISION);
    if (g_src_w <= 0 || g_src_h <= 0) {
        klog("视觉通道尺寸无效\n");
        return -1;
    }
    g_src_shape = dims_t{ 1, 3, (size_t)g_src_h, (size_t)g_src_w };
    g_src_bytes = (uint32_t)(3 * g_src_w * g_src_h);

    /* letterbox（与官方示例同一算法） */
    int in_w = (int)g_model_shape[3];
    int in_h = (int)g_model_shape[2];
    float ratiow = (float)in_w / (float)g_src_w;
    float ratioh = (float)in_h / (float)g_src_h;
    float ratio = ratiow < ratioh ? ratiow : ratioh;
    int new_w = (int)(ratio * g_src_w);
    int new_h = (int)(ratio * g_src_h);
    float dw = (float)(in_w - new_w) / 2.0f;
    float dh = (float)(in_h - new_h) / 2.0f;
    int top = 0;
    int bottom = (int)(dh * 2.0f + 0.1f);
    int left = 0;
    int right = (int)(dw * 2.0f - 0.1f);

    ai2d_datatype_t dtype{ ai2d_format::NCHW_FMT, ai2d_format::NCHW_FMT,
                           typecode_t::dt_uint8, typecode_t::dt_uint8 };
    ai2d_crop_param_t crop{ false, 0, 0, 0, 0 };
    ai2d_shift_param_t shift{ false, 0 };
    ai2d_pad_param_t pad{ true,
                          { { 0, 0 }, { 0, 0 }, { top, bottom }, { left, right } },
                          ai2d_pad_mode::constant, { 114, 114, 114 } };
    ai2d_resize_param_t resize{ true, ai2d_interp_method::tf_bilinear,
                                ai2d_interp_mode::half_pixel };
    ai2d_affine_param_t affine{ false, ai2d_interp_method::cv2_bilinear, 0, 0, 127, 1,
                                { 0.5f, 0.1f, 0.0f, 0.1f, 0.5f, 0.0f } };

    g_builder = new ai2d_builder(g_src_shape, g_model_shape, dtype, crop, shift, pad,
                                 resize, affine);
    auto br = g_builder->build_schedule();
    if (!br.is_ok()) {
        klog("ai2d build_schedule 失败\n");
        return -1;
    }

    /* 乒乓缓冲：输入 tensor 由 AI2D 写、KPU 读（同一份内存，零拷贝共用） */
    for (int i = 0; i < KPU_SLOTS; ++i) {
        auto res = host_runtime_tensor::create(typecode_t::dt_uint8, g_model_shape,
                                              hrt::pool_shared);
        if (!res.is_ok()) {
            klog("创建输入 tensor 失败\n");
            return -1;
        }
        g_slot[i].in = res.unwrap();
        g_slot[i].state = ST_FREE;
        g_slot[i].view.y = NULL;

        for (size_t k = 0; k < g_interp.outputs_size(); ++k) {
            auto od = host_runtime_tensor::create(g_interp.output_desc(k).datatype,
                                                 g_interp.output_shape(k), hrt::pool_shared);
            if (!od.is_ok()) {
                klog("创建输出 tensor %u 失败\n", (unsigned)k);
                return -1;
            }
            g_slot[i].out.push_back(od.unwrap());
        }
    }

    g_conf = g_cfg.kpu_conf > 0.0f ? g_cfg.kpu_conf : 0.35f;
    g_iou = g_cfg.kpu_iou > 0.0f ? g_cfg.kpu_iou : 0.45f;
    g_class_filter = g_cfg.kpu_class;
    g_stat_start_us = now_us();
    g_run = 1;

    if (pthread_create(&g_pre_tid, nullptr, pre_thread, nullptr) != 0) {
        klog("pre 线程创建失败\n");
        g_run = 0;
        return -1;
    }
    g_pre_started = 1;
    if (pthread_create(&g_kpu_tid, nullptr, kpu_thread, nullptr) != 0) {
        klog("kpu 线程创建失败\n");
        g_run = 0;
        pthread_cond_broadcast(&g_cond);
        pthread_join(g_pre_tid, nullptr);
        g_pre_started = 0;
        return -1;
    }
    g_kpu_started = 1;

    klog("已启用：kmodel=%s 输入 %dx%dx%d -> %ux%ux%d, nc=%d n_box=%d layout=%s, "
         "conf=%.2f iou=%.2f class=%d\n",
         g_cfg.kmodel, g_src_w, g_src_h, 3, (unsigned)g_model_shape[3],
         (unsigned)g_model_shape[2], (int)g_model_shape[1], g_nc, g_n_box,
         g_layout_cmajor ? "CxN" : "NxC", g_conf, g_iou, g_class_filter);
    return 0;
}

static int kpu_submit(const k_video_frame_info *frame, const frame_view_t *view, uint64_t seq)
{
    if (!g_run)
        return -1;

    pthread_mutex_lock(&g_lock);
    g_n_submit++;
    int idx = -1;
    for (int i = 0; i < KPU_SLOTS; ++i) {
        if (g_slot[i].state == ST_FREE) {
            idx = i;
            g_slot[i].state = ST_SCHED;
            break;
        }
    }
    if (idx < 0) {
        g_n_busy++;
        pthread_mutex_unlock(&g_lock);
        return 1;                     /* 忙：调用方自己 release 这帧 */
    }
    slot_t *s = &g_slot[idx];
    s->frame = *frame;                /* 所有权转移：本后端负责 release */
    s->view = *view;
    s->seq = seq;
    s->submit_us = view->dump_us ? view->dump_us : now_us();
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);
    return 0;
}

static int kpu_collect(detect_out_t *out)
{
    int got = 0;
    pthread_mutex_lock(&g_res_lock);
    if (g_res_cnt > 0) {
        *out = g_res[g_res_tail];
        g_res_tail = (g_res_tail + 1) % RESULT_RING;
        g_res_cnt--;
        got = 1;
    }
    pthread_mutex_unlock(&g_res_lock);
    return got;
}

static void kpu_stats_line(char *buf, size_t n)
{
    uint64_t done, ai2d_avg, kpu_avg, post_avg, lat_avg;
    uint64_t ai2d_max, kpu_max, post_max, lat_max;
    uint64_t busy, aerr, kerr;

    pthread_mutex_lock(&g_lock);
    done = g_n_done ? g_n_done : 1;
    ai2d_avg = g_t_ai2d_sum / done;
    kpu_avg = g_t_kpu_sum / done;
    post_avg = g_t_post_sum / done;
    lat_avg = g_t_lat_sum / done;
    ai2d_max = g_t_ai2d_max;
    kpu_max = g_t_kpu_max;
    post_max = g_t_post_max;
    lat_max = g_t_lat_max;
    busy = g_n_busy;
    aerr = g_n_ai2d_err;
    kerr = g_n_kpu_err;
    uint64_t nframes = g_n_done;
    /* 周期归零（统计口径与 status 周期一致） */
    g_t_ai2d_sum = g_t_kpu_sum = g_t_post_sum = g_t_lat_sum = 0;
    g_t_ai2d_max = g_t_kpu_max = g_t_post_max = g_t_lat_max = 0;
    g_n_submit = g_n_busy = g_n_ai2d_err = g_n_kpu_err = 0;
    pthread_mutex_unlock(&g_lock);

    uint64_t dur = now_us() - g_stat_start_us;
    g_stat_start_us = now_us();
    uint32_t fps = dur ? (uint32_t)(nframes * 1000000ull / dur) : 0;

    snprintf(buf, n,
             APP_NAME_STR ": kpu ai2d=%llu/%lluus kpu=%llu/%lluus post=%llu/%lluus "
             "lat=%llu/%lluus fps=%u done=%llu busy=%llu drop=%llu err(a2d=%llu,kpu=%llu)\n",
             (unsigned long long)ai2d_avg, (unsigned long long)ai2d_max,
             (unsigned long long)kpu_avg, (unsigned long long)kpu_max,
             (unsigned long long)post_avg, (unsigned long long)post_max,
             (unsigned long long)lat_avg, (unsigned long long)lat_max, fps,
             (unsigned long long)nframes, (unsigned long long)busy,
             (unsigned long long)g_n_dropped, (unsigned long long)aerr,
             (unsigned long long)kerr);
}

static void kpu_deinit(void)
{
    if (!g_run)
        return;

    g_run = 0;
    pthread_mutex_lock(&g_lock);
    pthread_cond_broadcast(&g_cond);
    pthread_mutex_unlock(&g_lock);

    if (g_pre_started) {
        pthread_join(g_pre_tid, nullptr);
        g_pre_started = 0;
    }
    if (g_kpu_started) {
        pthread_join(g_kpu_tid, nullptr);
        g_kpu_started = 0;
    }

    /* 收尾：把还在手里的 VICAP 帧还回去（只可能出现在 SCHED/READY 状态） */
    for (int i = 0; i < KPU_SLOTS; ++i) {
        if (g_slot[i].state == ST_SCHED || g_slot[i].state == ST_READY) {
            vicap_release(VICAP_CHN_VISION, &g_slot[i].frame);
            g_slot[i].state = ST_FREE;
        }
    }

    if (g_builder) {
        delete g_builder;
        g_builder = nullptr;
    }
}

static const detector_t g_detector_kpu = {
    "kpu",
    kpu_init,
    NULL,            /* 同步 run 不适用 */
    kpu_submit,
    kpu_collect,
    kpu_stats_line,
    kpu_deinit,
};

extern "C" const detector_t *detector_kpu(void)
{
    return &g_detector_kpu;
}
