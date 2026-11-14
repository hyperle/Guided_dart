#ifndef DART_SYSTEM_WRITE_PAUSE_H
#define DART_SYSTEM_WRITE_PAUSE_H

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define DART_DEFAULT_WRITE_PAUSE "/sdcard/app/control/write.pause"

/* Shared by the C launcher and C++ application. Any existing object pauses
 * writes. An inaccessible switch also pauses; only ENOENT permits writing. */
static inline const char* dart_write_pause_path(void) {
    const char* configured = getenv("DART_WRITE_PAUSE");
    return configured != NULL && configured[0] != '\0' ? configured : DART_DEFAULT_WRITE_PAUSE;
}

enum {
    DART_WRITE_SWITCH_ABSENT = 0,
    DART_WRITE_SWITCH_PRESENT = 1,
    /* RT-Smart's lstat() has returned 0 for the missing switch without filling
     * the stat buffer, in bursts, while recording (2026-11-14). Every real
     * object has a file type, so a zero st_mode answer carries no information. */
    DART_WRITE_SWITCH_UNKNOWN = 2,
};

static inline int dart_write_switch_state(const char* path) {
    struct stat status;
    memset(&status, 0, sizeof(status));
    if (lstat(path, &status) == 0)
        return (status.st_mode & S_IFMT) != 0 ? DART_WRITE_SWITCH_PRESENT : DART_WRITE_SWITCH_UNKNOWN;
    return errno == ENOENT ? DART_WRITE_SWITCH_ABSENT : DART_WRITE_SWITCH_PRESENT;
}

/* Stateless callers treat an unknown answer as paused. */
static inline int dart_writes_paused(const char* path) {
    return dart_write_switch_state(path) != DART_WRITE_SWITCH_ABSENT;
}

#endif
