/*
 * app_template —— 业务应用骨架示例（最小范式）
 *
 * ★ 你的业务代码放在 apps/<你的应用名>/ 目录下，本文件只是入口骨架，
 *   可直接删除并替换为你的业务代码。 ★
 *
 * 编译：bash scripts/build.sh
 * 产物：build/app_template/app_template  （riscv64 静态链接 ELF）
 * 部署：bash scripts/deploy.sh app_template <板子IP>
 * 板端：msh> /tmp/app_template
 */
#include <stdio.h>

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    printf("k230d rtos app template running.\n");
    return 0;
}
