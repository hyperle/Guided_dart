/* servo_probe —— Dart V1 四路舵机 PWM 只读取证工具
 *
 * 用途：在 dart_app 运行期间旁证"333 Hz 波形到底有没有出到连接器"。
 * 约束：只读。不写 PWM 寄存器，不 enable/disable 任何通道，不改 FPIOA 路由；
 *      默认不调用 drv_pwm_init/deinit（--pwm-init/--pwm-deinit 显式要求才调用），
 *      避免探针自己把另一个进程正在使用的四路输出掐掉。
 *
 * 读到的每一拍：
 *   drv_pwm_get_freq / drv_pwm_get_duty_ns   —— 驱动侧频率与高电平脉宽（ns）
 *   drv_fpioa_get_pin_cfg / get_pin_func     —— 引脚路由、输出/输入使能、MSC、PAD 电平
 *   --pad                                    —— 以 500 us 间隔采 40 次 PAD 电平，
 *                                               333 Hz/约 1.5 ms 高电平时应为高低混合
 *
 * 用法：
 *   servo_probe <log-path> [--delay-ms N] [--samples N] [--period-ms N]
 *                          [--pad] [--pwm-init] [--pwm-deinit] [--tag TEXT]
 * 例：
 *   servo_probe /data/app/logs/servo_probe_run.log --delay-ms 20000 --samples 20 --pad --tag during_app
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "drv_fpioa.h"
#include "drv_pwm.h"

#define CHANNEL_COUNT 4
#define PAD_SAMPLES 40

static const int g_channels[CHANNEL_COUNT] = {1, 2, 3, 5};
static const int g_pins[CHANNEL_COUNT] = {61, 7, 8, 59};

static void sleep_ms(unsigned ms) {
    struct timespec ts = {(time_t)(ms / 1000u), (long)(ms % 1000u) * 1000000L};
    while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {
    }
}

static long long now_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        return -1;
    return (long long)ts.tv_sec * 1000LL + (long long)(ts.tv_nsec / 1000000L);
}

static void dump_environment(FILE* out, const char* tag);

/* 包装运行模式：fork/exec 目标程序并把退出码、信号、stdout/stderr 落到日志。
 * 用途：旁证 dart_app 是不是"起来就死"，而 launcher 的 pid 复用判定看不到。
 * 命令行来自 --run-file：第一行可执行文件，其余每行一个参数。 */
static int run_file_mode(const char* log_path, const char* cmd_path, unsigned timeout_ms);

/* 入口标记模式：由开机自启直接拉起时执行。只记录现场，然后把进程交给真正的
 * launcher（--exec），因此不改变原有启动流程。 */
static void record_entry(FILE* out, int argc, char** argv, const char* tag) {
    fprintf(out, "entry tag=%s pid=%d", tag, (int)getpid());
    for (int i = 0; i < argc; ++i)
        fprintf(out, " argv[%d]=%s", i, argv[i]);
    fprintf(out, "\n");
    dump_environment(out, tag);
    fflush(out);
    fsync(fileno(out));
}

static int run_file_mode(const char* log_path, const char* cmd_path, unsigned timeout_ms) {
    FILE* cmd = fopen(cmd_path, "r");
    if (cmd == NULL) {
        FILE* err = fopen(log_path, "a");
        if (err != NULL) {
            fprintf(err, "run=error tag=run_file open=%s errno=%d\n", cmd_path, errno);
            fclose(err);
        }
        return 1;
    }
    char* argv[48];
    char storage[48][256];
    unsigned n = 0;
    while (n < 48 && fgets(storage[n], sizeof(storage[n]), cmd) != NULL) {
        char* line = storage[n];
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
            line[--len] = '\0';
        if (len == 0 || line[0] == '#')
            continue;
        argv[n] = line;
        ++n;
    }
    fclose(cmd);
    if (n == 0) {
        FILE* err = fopen(log_path, "a");
        if (err != NULL) {
            fprintf(err, "run=error tag=run_file empty=%s\n", cmd_path);
            fclose(err);
        }
        return 1;
    }
    argv[n] = NULL;

    FILE* out = fopen(log_path, "a");
    if (out == NULL)
        return 1;
    fprintf(out, "run=start tag=run_file path=%s args=%u pid=%d\n", argv[0], n - 1, (int)getpid());
    fflush(out);
    fsync(fileno(out));

    const pid_t pid = fork();
    if (pid < 0) {
        fprintf(out, "run=error tag=run_file fork_errno=%d\n", errno);
        fclose(out);
        return 1;
    }
    if (pid == 0) {
        (void)dup2(fileno(out), STDOUT_FILENO);
        (void)dup2(fileno(out), STDERR_FILENO);
        execv(argv[0], argv);
        _exit(127);
    }

    fprintf(out, "run=child tag=run_file child_pid=%d\n", (int)pid);
    fflush(out);
    fsync(fileno(out));

    unsigned waited_ms = 0;
    int status = 0;
    for (;;) {
        const pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid)
            break;
        if (timeout_ms != 0 && waited_ms >= timeout_ms) {
            fprintf(out, "run=alive tag=run_file child_pid=%d elapsed_ms=%u still_running=1\n",
                    (int)pid, waited_ms);
            fflush(out);
            fsync(fileno(out));
            fclose(out);
            return 0;
        }
        sleep_ms(50);
        waited_ms += 50;
    }

    if (WIFSIGNALED(status))
        fprintf(out, "run=exit tag=run_file child_pid=%d signal=%d elapsed_ms=%u\n", (int)pid,
                WTERMSIG(status), waited_ms);
    else
        fprintf(out, "run=exit tag=run_file child_pid=%d code=%d elapsed_ms=%u\n", (int)pid,
                WIFEXITED(status) ? WEXITSTATUS(status) : -1, waited_ms);
    fflush(out);
    fsync(fileno(out));
    fclose(out);
    return 0;
}

static void sample_pads(FILE* out, unsigned tag_slot) {
    unsigned high[CHANNEL_COUNT] = {0};
    for (unsigned n = 0; n < PAD_SAMPLES; ++n) {
        for (unsigned i = 0; i < CHANNEL_COUNT; ++i) {
            fpioa_iomux_cfg_t cfg = {0};
            if (drv_fpioa_get_pin_cfg(g_pins[i], &cfg.u.value) == 0 && cfg.u.bit.di)
                ++high[i];
        }
        struct timespec ts = {0, 500000L};
        while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {
        }
    }
    fprintf(out, "pad tag=%u samples=%u", tag_slot, (unsigned)PAD_SAMPLES);
    for (unsigned i = 0; i < CHANNEL_COUNT; ++i)
        fprintf(out, " io%d_high=%u", g_pins[i], high[i]);
    fprintf(out, "\n");
}

static void sample_registers(FILE* out, unsigned index, unsigned tag, int pwm_fd) {
    fprintf(out, "sample tag=%u index=%u t_ms=%lld pwm_fd=%d\n", tag, index, now_ms(), pwm_fd);
    for (unsigned i = 0; i < CHANNEL_COUNT; ++i) {
        uint32_t frequency = 0, pulse_ns = 0;
        fpioa_iomux_cfg_t cfg = {0};
        fpioa_func_t function = 0;
        errno = 0;
        const int rf = drv_pwm_get_freq(g_channels[i], &frequency);
        const int freq_errno = errno;
        errno = 0;
        const int rp = drv_pwm_get_duty_ns(g_channels[i], &pulse_ns);
        const int pulse_errno = errno;
        errno = 0;
        const int rc = drv_fpioa_get_pin_cfg(g_pins[i], &cfg.u.value);
        const int pin_errno = errno;
        errno = 0;
        const int rr = drv_fpioa_get_pin_func(g_pins[i], &function);
        const int func_errno = errno;
        fprintf(out,
                "  CH%u PWM%d IO%d hz=%u pulse_ns=%u rf=%d/%d rp=%d/%d"
                " pin_cfg=0x%08x(%d/%d) sel=%u oe=%u ie=%u di=%u msc=%u"
                " pin_func=%d(%d/%d)\n",
                i + 1, g_channels[i], g_pins[i], frequency, pulse_ns, rf, freq_errno, rp,
                pulse_errno, cfg.u.value, rc, pin_errno, (unsigned)cfg.u.bit.io_sel,
                (unsigned)cfg.u.bit.oe, (unsigned)cfg.u.bit.ie, (unsigned)cfg.u.bit.di,
                (unsigned)cfg.u.bit.msc, (int)function, rr, func_errno);
    }
}

/* 取证用：开机这一刻 /data 到底挂没挂、/sdcard 能不能写、入口文件在不在。 */
static void dump_environment(FILE* out, const char* tag) {
    const char* paths[] = {"/sdcard", "/sdcard/app", "/sdcard/app/launcher",
                           "/sdcard/app/dart_app", "/data", "/data/app", "/data/app/logs"};
    for (unsigned i = 0; i < sizeof(paths) / sizeof(paths[0]); ++i) {
        struct stat st;
        errno = 0;
        const int rc = stat(paths[i], &st);
        fprintf(out, "env tag=%s stat=%s rc=%d errno=%d isdir=%d size=%lld\n", tag, paths[i], rc,
                rc == 0 ? 0 : errno, rc == 0 && S_ISDIR(st.st_mode) ? 1 : 0,
                rc == 0 ? (long long)st.st_size : -1LL);
    }
    FILE* mounts = fopen("/proc/mounts", "r");
    if (mounts == NULL) {
        fprintf(out, "env tag=%s proc_mounts_open_errno=%d\n", tag, errno);
        return;
    }
    char line[512];
    while (fgets(line, sizeof(line), mounts) != NULL) {
        if (strstr(line, "sdcard") != NULL || strstr(line, "/data") != NULL)
            fprintf(out, "mounts tag=%s %s", tag, line);
    }
    fclose(mounts);
}

int main(int argc, char* argv[]) {
    const char* log_path = argc > 1 ? argv[1] : "/sdcard/servo_probe.log";
    unsigned delay_ms = 0, samples = 6, period_ms = 1000;
    int want_pad = 0, want_init = 0, want_deinit = 0, want_env = 0, want_entry = 0;
    const char* tag = "manual";
    const char* exec_path = NULL;
    const char* mark_path = NULL;
    const char* run_file = NULL;
    unsigned run_timeout_ms = 0;

    /* 无参数被拉起 = 开机入口模式：由固件把本程序当 launcher 启动。
     * 只记录现场，然后把进程交给真正的 launcher，不改变原有启动流程。 */
    if (argc == 1) {
        want_entry = 1;
        tag = "boot_entry";
        mark_path = "/sdcard/boot-marker.txt";
        exec_path = "/sdcard/app/launcher.real";
    }

    for (int i = 2; i < argc; ++i) {
        const char* a = argv[i];
        const int has_value = i + 1 < argc;
        if (strcmp(a, "--delay-ms") == 0 && has_value)
            delay_ms = (unsigned)strtoul(argv[++i], NULL, 10);
        else if (strcmp(a, "--samples") == 0 && has_value)
            samples = (unsigned)strtoul(argv[++i], NULL, 10);
        else if (strcmp(a, "--period-ms") == 0 && has_value)
            period_ms = (unsigned)strtoul(argv[++i], NULL, 10);
        else if (strcmp(a, "--pad") == 0)
            want_pad = 1;
        else if (strcmp(a, "--pwm-init") == 0)
            want_init = 1;
        else if (strcmp(a, "--pwm-deinit") == 0)
            want_deinit = 1;
        else if (strcmp(a, "--run-file") == 0 && has_value)
            run_file = argv[++i];
        else if (strcmp(a, "--run-timeout-ms") == 0 && has_value)
            run_timeout_ms = (unsigned)strtoul(argv[++i], NULL, 10);
        else if (strcmp(a, "--mark") == 0 && has_value)
            mark_path = argv[++i];
        else if (strcmp(a, "--entry") == 0)
            want_entry = 1;
        else if (strcmp(a, "--exec") == 0 && has_value)
            exec_path = argv[++i];
        else if (strcmp(a, "--env") == 0)
            want_env = 1;
        else if (strcmp(a, "--tag") == 0 && has_value)
            tag = argv[++i];
        else {
            fprintf(stderr, "servo_probe: 未知参数 %s\n", a);
            return 2;
        }
    }

    if (want_entry && mark_path != NULL)
        log_path = mark_path;
    if (run_file != NULL)
        return run_file_mode(log_path, run_file, run_timeout_ms);
    FILE* out = fopen(log_path, "a");
    if (out == NULL) {
        fprintf(stderr, "servo_probe: 打不开日志 %s: %s\n", log_path, strerror(errno));
        return 1;
    }
    fprintf(out, "probe=start tag=%s pid=%d read_only=1 delay_ms=%u samples=%u period_ms=%u\n",
            tag, (int)getpid(), delay_ms, samples, period_ms);
    if (want_env) {
        dump_environment(out, tag);
        fflush(out);
        fsync(fileno(out));
    }
    fflush(out);
    fsync(fileno(out));

    if (want_entry) {
        record_entry(out, argc, argv, tag);
        fclose(out);
        if (exec_path != NULL) {
            /* 只把 exec 目标交给 launcher：探针自己的参数不能转发，
             * 否则会被 launcher 当成清单路径解析。 */
            char* argv_out[2];
            argv_out[0] = (char*)exec_path;
            argv_out[1] = NULL;
            execv(exec_path, argv_out);
            FILE* err = fopen(log_path, "a");
            if (err != NULL) {
                fprintf(err, "entry tag=%s exec=%s failed errno=%d\n", tag, exec_path, errno);
                fflush(err);
                fsync(fileno(err));
                fclose(err);
            }
        }
        return 0;
    }

    if (delay_ms > 0) {
        fprintf(out, "probe=wait tag=%s delay_ms=%u\n", tag, delay_ms);
        fflush(out);
        fsync(fileno(out));
        sleep_ms(delay_ms);
    }

    errno = 0;
    const int pwm_fd = open("/dev/pwm", O_RDONLY);
    const int fd_errno = errno;

    int init_result = -1;
    if (want_init) {
        errno = 0;
        init_result = drv_pwm_init();
        fprintf(out, "probe=pwm_init tag=%s result=%d errno=%d\n", tag, init_result, errno);
        fflush(out);
        fsync(fileno(out));
    }

    for (unsigned index = 0; index < samples; ++index) {
        sample_registers(out, index, 0, pwm_fd);
        if (want_pad)
            sample_pads(out, index);
        fflush(out);
        fsync(fileno(out));
        if (index + 1 < samples)
            sleep_ms(period_ms);
    }

    if (pwm_fd >= 0)
        close(pwm_fd);
    if (want_deinit) {
        errno = 0;
        const int deinit = drv_pwm_deinit();
        fprintf(out, "probe=pwm_deinit tag=%s result=%d errno=%d\n", tag, deinit, errno);
    }
    fprintf(out, "probe=complete tag=%s dev_pwm_open_errno=%d\n", tag, pwm_fd >= 0 ? 0 : fd_errno);
    fflush(out);
    fsync(fileno(out));
    fclose(out);
    return 0;
}
