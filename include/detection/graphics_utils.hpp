#pragma once

#include <riscv_vector.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

namespace dart {

class GraphicsUtils {
public:
    static void binarize(const uint8_t *src, uint8_t *dst, uint32_t width, uint32_t height,
                        uint8_t threshold) {
        for (uint32_t row = 0; row < height; ++row)
            binarize_row(src + (size_t)row * width, dst + (size_t)row * width, width, threshold);
    }

    // 开机自检：向量结果与标量参考逐位比对，用来回答"这套固件上 RVV 到底能不能用"。
    // 必须存在：板端实测有两个互不相干的程序都在**首次执行 RVV** 时静默停住
    // （board_probe 的 RVV 基准、本程序首帧二值化），而它们在 RVV 之前的日志都正常。
    static bool selftest() {
        uint8_t src[256], bin[256], ref[256];

        for (uint32_t i = 0; i < sizeof(src); ++i)
            src[i] = (uint8_t)i;
        binarize(src, bin, (uint32_t)sizeof(src), 1, 128); // 走 RVV
        for (uint32_t i = 0; i < sizeof(src); ++i)
            ref[i] = src[i] > 128 ? 255 : 0; // 标量参考
        return memcmp(bin, ref, sizeof(src)) == 0;
    }
    
    static int save_pgm(const char *path, const uint8_t *data, uint32_t width, uint32_t height, uint32_t stride = 0) {
        if (stride == 0)
            stride = width;

        FILE *fp = fopen(path, "wb");
        if (fp == NULL)
            return -1;

        char hdr[64];
        int n = snprintf(hdr, sizeof(hdr), "P5\n%u %u\n255\n", (unsigned)width, (unsigned)height);
        int rc = 0;

        if (n <= 0 || fwrite(hdr, 1, (size_t)n, fp) != (size_t)n) {
            rc = -1;
        } else if (stride == width) {
            // 紧凑排列：整块一次写完，不逐行也不逐像素
            if (fwrite(data, 1, (size_t)width * height, fp) != (size_t)width * height)
                rc = -1;
        } else {
            for (uint32_t row = 0; row < height; ++row) {
                if (fwrite(data + (size_t)row * stride, 1, width, fp) != width) {
                    rc = -1;
                    break;
                }
            }
        }

        if (fclose(fp) != 0)
            rc = -1;
        return rc;
    }

    // PBM(P4)：1bit/像素（行末按字节对齐），二值图专用，体积是 PGM 的 1/8。
    // 入参必须是 binarize() 产出的 0/255 图；PBM 里 bit=1 表示黑(0)、bit=0 表示白(255)。
    static int save_pbm(const char *path, const uint8_t *bin, uint32_t width, uint32_t height, uint32_t stride = 0) {
        if (path == NULL || bin == NULL || width == 0 || height == 0)
            return -1;

        if (stride == 0)
            stride = width;

        const uint32_t row_bytes = (width + 7u) / 8u;
        uint8_t *packed = (uint8_t *)malloc(row_bytes);
        if (packed == NULL)
            return -1;

        FILE *fp = fopen(path, "wb");
        if (fp == NULL) {
            free(packed);
            return -1;
        }

        char hdr[64];
        int n = snprintf(hdr, sizeof(hdr), "P4\n%u %u\n", (unsigned)width, (unsigned)height);
        int rc = 0;

        if (n <= 0 || fwrite(hdr, 1, (size_t)n, fp) != (size_t)n)
            rc = -1;

        for (uint32_t row = 0; rc == 0 && row < height; ++row) {
            const uint8_t *src_row = bin + (size_t)row * stride;
            memset(packed, 0, row_bytes);
            for (uint32_t x = 0; x < width; ++x) {
                if (src_row[x] <= 127) // 暗像素 -> bit 1（PBM 的 1 是黑）
                    packed[x >> 3] |= (uint8_t)(0x80u >> (x & 7u));
            }
            if (fwrite(packed, 1, row_bytes, fp) != row_bytes)
                rc = -1;
        }

        free(packed);
        if (fclose(fp) != 0)
            rc = -1;
        return rc;
    }

    // BMP：8bit 灰度 + 灰度调色板，双击就能看（Windows 照片/画图、手机相册都认）。
    // 行按 4 字节对齐、像素自下而上存放，都是 BMP 的硬性规定。
    static int save_bmp(const char *path, const uint8_t *data, uint32_t width, uint32_t height, uint32_t stride = 0) {
        if (path == NULL || data == NULL || width == 0 || height == 0)
            return -1;

        if (stride == 0)
            stride = width;

        const uint32_t row_bytes = (width + 3u) & ~3u;    // 每行 4 字节对齐
        const uint32_t pix_bytes = row_bytes * height;
        const uint32_t hdr_bytes = 14u + 40u + 256u * 4u; // 文件头 + DIB 头 + 灰度调色板

        uint8_t hdr[hdr_bytes];
        memset(hdr, 0, sizeof(hdr));

        hdr[0] = 'B';
        hdr[1] = 'M';
        put_u32_le(hdr + 2, hdr_bytes + pix_bytes); // 文件总大小
        put_u32_le(hdr + 10, hdr_bytes);            // 像素数据偏移
        put_u32_le(hdr + 14, 40);                   // DIB 头大小
        put_u32_le(hdr + 18, width);
        put_u32_le(hdr + 22, height); // 正数 = 自下而上
        put_u16_le(hdr + 26, 1);      // planes
        put_u16_le(hdr + 28, 8);      // bpp
        put_u32_le(hdr + 34, pix_bytes);
        put_u32_le(hdr + 38, 2835); // 96 DPI
        put_u32_le(hdr + 42, 2835);
        put_u32_le(hdr + 46, 256); // 调色板颜色数

        for (uint32_t i = 0; i < 256; ++i) { // 灰度调色板，每项 BGRA
            hdr[54 + i * 4 + 0] = (uint8_t)i;
            hdr[54 + i * 4 + 1] = (uint8_t)i;
            hdr[54 + i * 4 + 2] = (uint8_t)i;
        }

        FILE *fp = fopen(path, "wb");
        if (fp == NULL)
            return -1;

        int rc = 0;
        if (fwrite(hdr, 1, sizeof(hdr), fp) != sizeof(hdr))
            rc = -1;

        if (rc == 0) {
            uint8_t *row_buf = (uint8_t *)calloc(1, row_bytes); // 零填充：行尾补零恒定成立
            if (row_buf == NULL) {
                rc = -1;
            } else {
                for (uint32_t y = 0; y < height; ++y) {
                    const uint8_t *src_row = data + (size_t)(height - 1u - y) * stride; // 自下而上
                    memcpy(row_buf, src_row, width);
                    if (fwrite(row_buf, 1, row_bytes, fp) != row_bytes) {
                        rc = -1;
                        break;
                    }
                }
                free(row_buf);
            }
        }

        if (fclose(fp) != 0)
            rc = -1;
        return rc;
    }

    // ======================== 存图（按扩展名分派） ========================

    // 灰度/原始 Y 平面 -> .pgm .bmp
    static int save_gray(const char *path, const uint8_t *data, uint32_t width, uint32_t height, uint32_t stride = 0) {
        const char *ext = ext_of(path);
        if (ext == NULL)
            return -1;

        if (strcasecmp(ext, ".pgm") == 0)
            return save_pgm(path, data, width, height, stride);
        if (strcasecmp(ext, ".bmp") == 0)
            return save_bmp(path, data, width, height, stride);
        return -1; // 不支持的扩展名
    }

    // binarize() 产出的 0/255 二值图 -> .pbm（1bit，最小）.pgm .bmp
    static int save_binary(const char *path, const uint8_t *bin, uint32_t width, uint32_t height, uint32_t stride = 0) {
        const char *ext = ext_of(path);
        if (ext == NULL)
            return -1;

        if (strcasecmp(ext, ".pbm") == 0)
            return save_pbm(path, bin, width, height, stride);
        if (strcasecmp(ext, ".pgm") == 0)
            return save_pgm(path, bin, width, height, stride);
        if (strcasecmp(ext, ".bmp") == 0)
            return save_bmp(path, bin, width, height, stride);
        return -1; // 不支持的扩展名
    }

    //h265编码

private:
    GraphicsUtils() = default;
    ~GraphicsUtils() = default;

    // 单行二值化：LMUL=8，C908 一次吃 128 字节
    static void binarize_row(const uint8_t *src, uint8_t *dst, uint32_t width, uint8_t threshold) {
        uint32_t x = 0;
        while (x < width) {
            size_t vl = vsetvl_e8m8((size_t)(width - x));
            vuint8m8_t v = vle8_v_u8m8(src + x, vl);
            // 必须用 vmsgtu（无符号）：vmsgt 是有符号比较，Y > 127 会被判成负数而结果全错
            vbool1_t m = vmsgtu_vx_u8m8_b1(v, threshold, vl);
            // vmerge.vxm 的语义是 vd[i] = mask[i] ? rs1(标量) : vs2(向量)，命中 255、否则 0；
            // 注意 GCC 的 vmerge_* 把 mask 放在第一个参数
            vuint8m8_t r = vmerge_vxm_u8m8(m, vmv_v_x_u8m8(0, vl), 255, vl);
            vse8_v_u8m8(dst + x, r, vl);
            x += (uint32_t)vl;
        }
    }

    // 取扩展名（含点）；无扩展名返回 NULL
    static const char *ext_of(const char *path) {
        if (path == NULL)
            return NULL;

        const char *dot = strrchr(path, '.');
        const char *slash = strrchr(path, '/');
        if (dot == NULL || (slash != NULL && dot < slash))
            return NULL;
        return dot;
    }

    // BMP 头按小端手工拼字节：不依赖结构体 packing，宿主机/板端行为一致
    static void put_u16_le(uint8_t *p, uint16_t v) {
        p[0] = (uint8_t)(v & 0xffu);
        p[1] = (uint8_t)((v >> 8) & 0xffu);
    }

    static void put_u32_le(uint8_t *p, uint32_t v) {
        p[0] = (uint8_t)(v & 0xffu);
        p[1] = (uint8_t)((v >> 8) & 0xffu);
        p[2] = (uint8_t)((v >> 16) & 0xffu);
        p[3] = (uint8_t)((v >> 24) & 0xffu);
    }
};

} //namespace dart