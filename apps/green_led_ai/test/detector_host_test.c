#include <stdio.h>
#include <string.h>
#include <math.h>
#include "app.h"
#include "detect.h"

/* RGB(BT.601 视频范围) -> NV12，用于造测试帧 */
static void rgb2yuv(int r,int g,int b,uint8_t*Y,uint8_t*U,uint8_t*V){
    int y = (( 66*r + 129*g +  25*b + 128) >> 8) + 16;
    int u = ((-38*r -  74*g + 112*b + 128) >> 8) + 128;
    int v = ((112*r -  94*g -  18*b + 128) >> 8) + 128;
    *Y=(uint8_t)(y<0?0:(y>255?255:y)); *U=(uint8_t)(u<0?0:(u>255?255:u)); *V=(uint8_t)(v<0?0:(v>255?255:v));
}

static uint8_t frame[640*480*3/2];
static void fill(uint8_t Y,uint8_t U,uint8_t V){ memset(frame,Y,640*480); memset(frame+640*480,V,640*480/4*2);
    for(int i=0;i<640*480/4;i++){ frame[640*480+i*2]=U; frame[640*480+i*2+1]=V; } }
static void blob(int cx,int cy,int rad,uint8_t Y,uint8_t U,uint8_t V){
    for(int y=cy-rad;y<=cy+rad;y++) for(int x=cx-rad;x<=cx+rad;x++){
        if((x-cx)*(x-cx)+(y-cy)*(y-cy) > rad*rad) continue;
        if(x<0||y<0||x>=640||y>=480) continue;
        frame[(size_t)y*640+x]=Y;
        size_t uv=(size_t)(y/2)*640 + (x/2)*2;
        frame[640*480+uv]=U; frame[640*480+uv+1]=V;
    }
}

int main(void){
    uint8_t gY,gU,gV,bY,bU,bV;
    rgb2yuv(0,255,0,&gY,&gU,&gV);      /* 纯绿 */
    rgb2yuv(128,128,128,&bY,&bU,&bV);  /* 中灰背景 */
    printf("green NV12=(%u,%u,%u) gray=(%u,%u,%u)\n",gY,gU,gV,bY,bU,bV);

    g_cfg.lab_l_min=12; g_cfg.lab_a_max=-20; g_cfg.lab_b_min=8;
    g_cfg.roi_x=160; g_cfg.roi_y=120; g_cfg.roi_w=320; g_cfg.roi_h=240;
    g_cfg.step_x=2; g_cfg.step_y=2; g_cfg.pix_min=50; g_cfg.miss_full_scan=3;
    g_cfg.vis_w=640; g_cfg.vis_h=480;

    const detector_t *d = detector_get("color");
    if (!d || d->init()!=0) { printf("init failed\n"); return 1; }

    frame_view_t f = { frame, frame+640*480, 640, 640, 480, 1 };

    /* 1) 纯灰画面：应未命中 */
    fill(bY,bU,bV);
    detect_out_t o; d->run(&f,0,&o);
    printf("TEST1 gray: cx=%d cy=%d green_px=%u blobs=%u t=%uus %s\n",
           o.cx,o.cy,o.green_px,o.blobs,o.t_us,(o.cx<0)?"PASS":"FAIL");

    /* 2) 灰底 + 半径 20 绿灯（中心 320,240，正好 ROI 中心）：应命中 */
    fill(bY,bU,bV);
    blob(320,240,20,gY,gU,gV);
    d->run(&f,0,&o);
    printf("TEST2 green blob: cx=%d cy=%d px=%u blobs=%u green_px=%u probe_rgb=(%d,%d,%d) lab=(%d,%d,%d) t=%uus %s\n",
           o.cx,o.cy,o.px,o.blobs,o.green_px,o.probe_r,o.probe_g,o.probe_b,
           o.probe_L,o.probe_A,o.probe_B,o.t_us,
           (o.cx>=310&&o.cx<=330&&o.cy>=230&&o.cy<=250&&o.blobs>=1)?"PASS":"FAIL");

    /* 3) 全画面扫描（step 1）与 ROI+step2 结果应一致 */
    g_cfg.roi_w=0; g_cfg.roi_h=0; g_cfg.step_x=1; g_cfg.step_y=1;
    d->init();
    d->run(&f,1,&o);
    printf("TEST3 full scan step1: cx=%d cy=%d px=%u blobs=%u green_px=%u t=%uus %s\n",
           o.cx,o.cy,o.px,o.blobs,o.green_px,o.t_us,
           (o.cx>=310&&o.cx<=330&&o.cy>=230&&o.cy<=250)?"PASS":"FAIL");

    /* 4) 性能参考：全画面 step1 的纯检测耗时（宿主机，仅作量级参考） */
    double best=1e9; for(int i=0;i<20;i++){ d->run(&f,1,&o); if(o.t_us<best) best=o.t_us; }
    printf("TEST4 full 640x480 step1 detect best=%uus (host scalar)\n",(int)best);

    d->deinit();
    return 0;
}
