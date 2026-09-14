/*
 * rec_dir 宿主机单元测试：会话编号 + 容量淘汰
 *
 * 这段逻辑会真的删掉用户的录像文件，所以必须验证：
 *   1) 会话编号取最大 idx + 1（不重复、不覆盖已有会话）
 *   2) 只在总量超上限时才删，且从最旧（idx 最小）开始删
 *   3) 同一会话的 .h264 与 .csv 成对删除，不留孤儿
 *   4) 不碰非 rec_<数字>.(h264|csv) 的文件（日志、ppm 等）
 *   5) 上限调大后不再删除（幂等）
 *
 * 运行: bash apps/green_led_ai/test/run_host_test.sh
 */
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "rec_dir.h"

#define TDIR "/tmp/green_led_ai_recdir_test"

static int fails;

static void check(const char *what, int ok)
{
    printf("%-62s %s\n", what, ok ? "PASS" : "FAIL");
    if (!ok)
        fails++;
}

static void write_file(const char *dir, const char *name, size_t bytes)
{
    char path[600];
    snprintf(path, sizeof(path), "%s/%s", dir, name);
    FILE *f = fopen(path, "wb");
    if (!f)
        return;
    /* 写字节数用 ftruncate 更快：先建文件再截断 */
    fclose(f);
    truncate(path, (off_t)bytes);
}

static int exists(const char *dir, const char *name)
{
    char path[600];
    snprintf(path, sizeof(path), "%s/%s", dir, name);
    return access(path, F_OK) == 0;
}

static void clean_dir(void)
{
    DIR *d = opendir(TDIR);
    if (d) {
        struct dirent *e;
        while ((e = readdir(d)) != NULL) {
            if (e->d_name[0] == '.')
                continue;
            char path[600];
            snprintf(path, sizeof(path), "%s/%s", TDIR, e->d_name);
            unlink(path);
        }
        closedir(d);
    }
}

int main(void)
{
    mkdir(TDIR, 0777);
    clean_dir();

    /* ---- 1) 空目录：下一个编号应为 1 ---- */
    check("TEST1 空目录 next_index == 1", rec_dir_next_index(TDIR) == 1);

    /*
     * 造 5 段会话，每段 h264=100KB、csv=4KB（合计 104KB/会话），
     * 另放两个「不该被删/不该被统计」的文件。
     */
    for (unsigned i = 1; i <= 5; ++i) {
        char h[64], c[64];
        snprintf(h, sizeof(h), "rec_%04u.h264", i);
        snprintf(c, sizeof(c), "rec_%04u.csv", i);
        write_file(TDIR, h, 100 * 1024);
        write_file(TDIR, c, 4 * 1024);
    }
    write_file(TDIR, "frame_00001.ppm", 900 * 1024);   /* 旧工程导出的整帧 PPM */
    write_file(TDIR, "notes.txt", 1024);

    check("TEST2 next_index == 6（不覆盖已有会话）", rec_dir_next_index(TDIR) == 6);
    check("TEST3 总字节只统计会话文件 (5*104KB)",
          rec_dir_total_bytes(TDIR) == 5 * 104 * 1024);

    /* ---- 2) 上限 300KB：应删到只剩 2 段（208KB <= 300KB < 312KB） ---- */
    int removed = rec_dir_evict(TDIR, 300 * 1024);
    check("TEST4 cap=300KB 删除了 3 段最旧会话", removed == 3);
    check("TEST5 rec_0001/0002/0003 已删除",
          !exists(TDIR, "rec_0001.h264") && !exists(TDIR, "rec_0001.csv") &&
              !exists(TDIR, "rec_0002.h264") && !exists(TDIR, "rec_0002.csv") &&
              !exists(TDIR, "rec_0003.h264") && !exists(TDIR, "rec_0003.csv"));
    check("TEST6 rec_0004/0005 保留（成对）",
          exists(TDIR, "rec_0004.h264") && exists(TDIR, "rec_0004.csv") &&
              exists(TDIR, "rec_0005.h264") && exists(TDIR, "rec_0005.csv"));
    check("TEST7 未触碰 ppm/txt",
          exists(TDIR, "frame_00001.ppm") && exists(TDIR, "notes.txt"));
    check("TEST8 淘汰后总量 <= 上限", rec_dir_total_bytes(TDIR) <= 300 * 1024);

    /* ---- 3) 幂等：上限放宽后不再删 ---- */
    removed = rec_dir_evict(TDIR, 1024 * 1024);
    check("TEST9 cap 放宽后不再删除", removed == 0 && exists(TDIR, "rec_0004.h264"));

    /* ---- 4) 畸形文件名不应被统计/删除 ---- */
    write_file(TDIR, "rec_abc.h264", 50 * 1024);
    write_file(TDIR, "rec_0006.h26", 50 * 1024);
    write_file(TDIR, "xrec_0007.h264", 50 * 1024);
    long long total = rec_dir_total_bytes(TDIR);
    check("TEST10 畸形 rec_* 文件不计入统计", total == 2 * 104 * 1024);
    removed = rec_dir_evict(TDIR, 1);          /* 极限：只剩最小可能 */
    check("TEST11 畸形文件永不被删",
          exists(TDIR, "rec_abc.h264") && exists(TDIR, "rec_0006.h26") &&
              exists(TDIR, "rec_x264") == 0 && exists(TDIR, "xrec_0007.h264"));

    /* ---- 5) 只剩 .csv（异常断电）也不会崩 ---- */
    clean_dir();
    write_file(TDIR, "rec_0009.csv", 10 * 1024);
    check("TEST12 只有 csv 的孤儿会话可被淘汰",
          rec_dir_evict(TDIR, 1) == 1 && !exists(TDIR, "rec_0009.csv"));

    /* ---- 6) 不存在的目录 ---- */
    check("TEST13 目录不存在时返回 -1",
          rec_dir_evict("/tmp/no_such_dir_green_led_ai", 1) == -1);

    clean_dir();
    rmdir(TDIR);

    printf("rec_dir host test: %s (%d failures)\n", fails ? "FAIL" : "ALL PASS", fails);
    return fails ? 1 : 0;
}
