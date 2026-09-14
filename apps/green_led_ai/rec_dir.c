/*
 * rec_dir —— 实现
 */
#include "rec_dir.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define MAX_DIR_FILES 512

/* 只接受 rec_<数字>.h264 / rec_<数字>.csv；返回 0 表示是会话文件 */
static int session_file(const char *name, uint32_t *idx)
{
    if (strncmp(name, "rec_", 4) != 0)
        return -1;

    const char *p = name + 4;
    if (*p < '0' || *p > '9')
        return -1;

    unsigned long v = strtoul(p, (char **)&p, 10);
    if (v > 0xFFFFFFFFul)
        return -1;

    if (strcmp(p, ".h264") != 0 && strcmp(p, ".csv") != 0)
        return -1;

    *idx = (uint32_t)v;
    return 0;
}

uint32_t rec_dir_next_index(const char *dir)
{
    uint32_t max_idx = 0;
    DIR *d = opendir(dir);
    if (!d)
        return 1;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        uint32_t idx;
        if (session_file(e->d_name, &idx) == 0 && idx > max_idx)
            max_idx = idx;
    }
    closedir(d);
    return max_idx + 1;
}

long long rec_dir_total_bytes(const char *dir)
{
    DIR *d = opendir(dir);
    if (!d)
        return -1;

    long long total = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        uint32_t idx;
        if (session_file(e->d_name, &idx) != 0)
            continue;
        char path[560];
        snprintf(path, sizeof(path), "%s/%s", dir, e->d_name);
        struct stat st;
        if (stat(path, &st) == 0 && S_ISREG(st.st_mode))
            total += (long long)st.st_size;
    }
    closedir(d);
    return total;
}

int rec_dir_evict(const char *dir, uint64_t cap_bytes)
{
    typedef struct {
        uint32_t idx;
        uint64_t size;
    } ent_t;
    static ent_t ents[MAX_DIR_FILES];
    int n = 0;
    long long total = 0;

    DIR *d = opendir(dir);
    if (!d)
        return -1;

    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        uint32_t idx;
        if (session_file(e->d_name, &idx) != 0)
            continue;
        char path[560];
        snprintf(path, sizeof(path), "%s/%s", dir, e->d_name);
        struct stat st;
        if (stat(path, &st) != 0 || !S_ISREG(st.st_mode))
            continue;
        total += (long long)st.st_size;
        if (n < MAX_DIR_FILES) {
            ents[n].idx = idx;
            ents[n].size = (uint64_t)st.st_size;
            n++;
        }
    }
    closedir(d);

    if (n == 0 || total <= (long long)cap_bytes)
        return 0;

    /* 按 idx 升序插入排序（旧 -> 新） */
    for (int i = 1; i < n; i++) {
        ent_t t = ents[i];
        int j = i - 1;
        while (j >= 0 && ents[j].idx > t.idx) {
            ents[j + 1] = ents[j];
            j--;
        }
        ents[j + 1] = t;
    }

    /* 同名会话的 .h264/.csv 合并为一条，保证成对删除 */
    int m = 0;
    for (int i = 0; i < n; i++) {
        if (m > 0 && ents[m - 1].idx == ents[i].idx)
            ents[m - 1].size += ents[i].size;
        else
            ents[m++] = ents[i];
    }

    int removed = 0;
    for (int i = 0; i < m && total > (long long)cap_bytes; i++) {
        char h[600], c[600];
        snprintf(h, sizeof(h), "%s/rec_%04u.h264", dir, ents[i].idx);
        snprintf(c, sizeof(c), "%s/rec_%04u.csv", dir, ents[i].idx);

        /* 先确认这对文件真的被删掉（unlink 失败也不把大小从 total 里扣掉） */
        int ok = 0;
        if (unlink(h) == 0)
            ok = 1;
        if (unlink(c) == 0)
            ok = 1;
        if (!ok)
            continue;

        total -= (long long)ents[i].size;
        removed++;
    }
    return removed;
}
