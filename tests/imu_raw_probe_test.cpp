#include <cstddef>
#include <cassert>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "drv_spi.h"

int imu_raw_probe_main(int argc, char** argv);

static int creates = 0, writes = 0, transfers = 0, destroys = 0, pauses = 0, wake_pauses = 0;
static bool bus_failure = false, read_failure = false, sleep_failure = false;
static bool bank_failure = false, wake_failure = false, wake_short_write = false;
static bool wake_sleep_failure = false, wake_sleep_interrupt = false, wake_stop = false;
static bool wake_read_failure = false, wake_mismatch = false, wake_complete = false;
static bool power_save_read_failure = false, power_save_mismatch = false;
static unsigned power_save_reads = 0, power_save_writes = 0;
static std::uint8_t apex_config = 0xfe, smd_config = 0xab;
static drv_spi_inst_t handle = reinterpret_cast<drv_spi_inst_t>(0x1234);

extern "C" int drv_spi_inst_create(
    int bus, bool active_low, int mode, std::uint32_t hz, std::uint8_t bits,
    int cs, std::uint8_t lines, drv_spi_inst_t* out) {
    assert(bus == 1 && active_low && mode == SPI_HAL_MODE_3 && hz == 1'000'000);
    assert(bits == 8 && cs == 14 && lines == SPI_HAL_DATA_LINE_1);
    ++creates;
    *out = handle; // Also model the SDK's dangling output on creation failure.
    return bus_failure ? -1 : 0;
}

extern "C" int drv_spi_write(
    drv_spi_inst_t spi, const void* tx, std::size_t size, bool release_cs) {
    assert(spi == handle && release_cs && size == 2);
    const auto* bytes = static_cast<const std::uint8_t*>(tx);
    if (bytes[0] == 0x56 || bytes[0] == 0x57) {
        assert(writes == 3 && !wake_complete && transfers == 0);
        assert(bytes[1] == (bytes[0] == 0x56 ? 0x0a : 0xa8));
        ++power_save_writes;
        if (!power_save_mismatch)
            (bytes[0] == 0x56 ? apex_config : smd_config) = bytes[1];
        return 2;
    }
    assert(writes < 4 && transfers == 0 && wake_pauses == 0);
    constexpr std::uint8_t commands[4][2]{{0x76, 0x00}, {0x4e, 0x00},
        {0x16, 0x00}, {0x4e, 0x0f}};
    assert(bytes[0] == commands[writes][0] && bytes[1] == commands[writes][1]);
    ++writes;
    if ((bank_failure && writes == 1) || (wake_failure && writes == 4))
        return -1;
    return wake_short_write && writes == 4 ? 1 : static_cast<int>(size);
}

extern "C" int drv_spi_transfer(
    drv_spi_inst_t spi, const void* tx, void* rx, std::size_t size, bool release_cs) {
    assert(spi == handle && release_cs);
    const auto* bytes = static_cast<const std::uint8_t*>(tx);
    if (bytes[0] == 0xf6) {
        assert(writes == 1 && size == 2 && !wake_complete);
        std::memset(rx, 0, size);
        return 2;
    }
    if (bytes[0] == 0xd6 || bytes[0] == 0xd7) {
        assert(writes == 3 && size == 2 && !wake_complete);
        ++power_save_reads;
        if (power_save_read_failure)
            return -1;
        std::memset(rx, 0, size);
        static_cast<std::uint8_t*>(rx)[1] = bytes[0] == 0xd6 ? apex_config : smd_config;
        return 2;
    }
    // A sensor starting asleep must receive both wake writes and the full
    // startup wait before even the first register read.
    assert(writes == 4 && wake_complete);
    const bool power = transfers == 0;
    const bool id = !power && (transfers - 1) % 2 == 0;
    assert(size == (power || id ? 2 : 15));
    assert(bytes[0] == (power ? 0xce : id ? 0xf5 : 0x9d));
    for (std::size_t i = 1; i < size; ++i)
        assert(bytes[i] == 0);
    // Unrecognized IDs, all-zero/all-one frames and read failures must not
    // prevent subsequent polling after the wake sequence.
    std::memset(rx, transfers < 3 ? 0 : 0xff, size);
    if (power)
        static_cast<std::uint8_t*>(rx)[1] = wake_mismatch ? 0x07 : 0x0f;
    ++transfers;
    return (power ? wake_read_failure : read_failure) ? -1 : static_cast<int>(size);
}

extern "C" void drv_spi_inst_destroy(drv_spi_inst_t* spi) {
    assert(*spi == handle && !bus_failure);
    *spi = nullptr;
    ++destroys;
}

extern "C" int __wrap_nanosleep(const timespec* requested, timespec* remaining) {
    if (!bus_failure && writes == 1) {
        assert(requested->tv_sec == 0 && requested->tv_nsec == 50'000'000);
        return 0; // Allow an IMU left ON by CPU reset to satisfy minimum ON time.
    }
    if (!bus_failure && writes == 2) {
        assert(requested->tv_sec == 0 && requested->tv_nsec == 200'000'000);
        return 0; // Both sensors are explicitly OFF before FIFO configuration.
    }
    if (!bus_failure && !wake_complete) {
        assert(writes == 4 && transfers == 0);
        assert(requested->tv_sec == 0);
        assert(requested->tv_nsec == (wake_pauses == 0 ? 50'000'000 : 20'000'000));
        ++wake_pauses;
        if (wake_sleep_failure) {
            errno = EINVAL;
            return -1;
        }
        if (wake_stop) {
            std::raise(SIGTERM);
            errno = EINTR;
            return -1;
        }
        if (wake_sleep_interrupt && wake_pauses == 1) {
            *remaining = timespec{0, 20'000'000};
            errno = EINTR;
            return -1;
        }
        wake_complete = true;
        return 0;
    }
    assert(requested->tv_sec == (bus_failure ? 1 : 0));
    assert(requested->tv_nsec == (bus_failure ? 0 : 10'000'000));
    ++pauses;
    if (sleep_failure) {
        errno = EINVAL;
        return -1;
    }
    if (pauses == 3) {
        std::raise(SIGTERM);
        errno = EINTR;
        return -1;
    }
    return 0;
}

int main(int argc, char** argv) {
    assert(argc == 2);
    bus_failure = std::strcmp(argv[1], "bus_failure") == 0;
    read_failure = std::strcmp(argv[1], "read_failure") == 0;
    sleep_failure = std::strcmp(argv[1], "sleep_failure") == 0;
    bank_failure = std::strcmp(argv[1], "bank_failure") == 0;
    wake_failure = std::strcmp(argv[1], "wake_failure") == 0;
    wake_short_write = std::strcmp(argv[1], "wake_short_write") == 0;
    wake_sleep_failure = std::strcmp(argv[1], "wake_sleep_failure") == 0;
    wake_sleep_interrupt = std::strcmp(argv[1], "wake_sleep_interrupt") == 0;
    wake_stop = std::strcmp(argv[1], "wake_stop") == 0;
    wake_read_failure = std::strcmp(argv[1], "wake_read_failure") == 0;
    wake_mismatch = std::strcmp(argv[1], "wake_mismatch") == 0;
    power_save_read_failure = std::strcmp(argv[1], "power_save_read_failure") == 0;
    power_save_mismatch = std::strcmp(argv[1], "power_save_mismatch") == 0;
    const bool config_failure = power_save_read_failure || power_save_mismatch;
    const bool write_failed = bank_failure || wake_failure || wake_short_write;
    const bool no_reads = bus_failure || write_failed || config_failure || wake_sleep_failure || wake_stop;
    const bool failed = sleep_failure || write_failed || config_failure || wake_sleep_failure || wake_read_failure || wake_mismatch;
    char log_path[] = "/tmp/dart-imu-raw-test-XXXXXX";
    const int fd = mkstemp(log_path);
    assert(fd >= 0);
    close(fd);
    char name[] = "imu_raw_probe";
    char* arguments[] = {name, log_path, nullptr};
    assert(imu_raw_probe_main(2, arguments) == (failed ? 2 : 0));
    assert(creates == (bus_failure ? 3 : 1));
    assert(writes == (bus_failure ? 0 : bank_failure ? 1 : config_failure ? 3 : 4));
    assert(wake_pauses == (bus_failure || write_failed || config_failure ? 0 : wake_sleep_interrupt ? 2 : 1));
    assert(transfers == (no_reads ? 0 : wake_read_failure || wake_mismatch ? 1 : sleep_failure ? 3 : 7));
    assert(destroys == (bus_failure ? 0 : 1));
    assert(pauses == (bus_failure ? 3 : no_reads || wake_read_failure || wake_mismatch ? 0 : sleep_failure ? 1 : 3));
    FILE* log = std::fopen(log_path, "r");
    assert(log != nullptr);
    char text[2048]{};
    assert(std::fread(text, 1, sizeof(text) - 1, log) > 0);
    assert(std::strstr(text, "sensor_init=low_noise_poll_v7") != nullptr);
    if (bus_failure)
        assert(std::strstr(text, "op=bus result=retry") != nullptr);
    else {
        assert(std::strstr(text, "op=bus result=ready") != nullptr);
        assert(std::strstr(text, "op=wake_write register=0x76 value=0x00") != nullptr);
        if (!bank_failure && !config_failure)
            assert(std::strstr(text, "op=wake_write register=0x4e value=0x0f") != nullptr);
        if (!bank_failure && !config_failure) {
            assert(power_save_reads == 4 && power_save_writes == 2);
            assert(apex_config == 0x0a && smd_config == 0xa8);
        }
        if (write_failed)
            assert(std::strstr(text, "result=error") != nullptr);
        if (wake_sleep_failure)
            assert(std::strstr(text, "op=wake result=wait_error") != nullptr);
        if (!no_reads) {
            assert(std::strstr(text, "op=wake wait_ms=50") != nullptr);
            assert(std::strstr(text, wake_read_failure || wake_mismatch ? "verified=0" : "verified=1") != nullptr);
            if (!wake_read_failure && !wake_mismatch) {
                assert(std::strstr(text, "raw=0000000000000000000000000000") != nullptr);
                assert(std::strstr(text, read_failure ? "errors=2 id_rc=-1" : "errors=0 id_rc=2") != nullptr);
            } else
                assert(std::strstr(text, "raw=") == nullptr);
        } else
            assert(std::strstr(text, "raw=") == nullptr);
        assert(std::strstr(text, "op=stop") != nullptr);
    }
    std::fclose(log);
    assert(unlink(log_path) == 0);
}
