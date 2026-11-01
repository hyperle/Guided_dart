module;
#include <expected>
#include <memory>
#include <new>
#include <utility>

#include "spi/iim42652.h"

export module dart.board.k230;

#ifdef DART_APP_IMU_K230
import dart.imu.k230.iim42652;
#endif
#ifdef DART_APP_PWM_K230
import dart.pwm.k230.output;
#endif
#ifdef DART_APP_WIFI_K230
import dart.streaming.k230.transport;
#endif
import dart.streaming.interface;
import dart.camera.k230.camera;
import dart.logging.logger;
import dart.runtime.imu.session;
import dart.storage.directories;
import dart.system.device_services;

export namespace dart::board {

std::unique_ptr<streaming::ITransport> create_wifi_transport() noexcept {
#ifdef DART_APP_WIFI_K230
    return std::unique_ptr<streaming::ITransport>{new (std::nothrow) streaming::k230::Transport};
#else
    return nullptr;
#endif
}

std::expected<std::unique_ptr<pwm::IPwm>, pwm::Error> create_pwm() noexcept {
#ifdef DART_APP_PWM_K230
    auto* device = new (std::nothrow) pwm::k230::Output;
    if (device == nullptr)
        return std::unexpected(pwm::Error{"allocate", -1, -1});
    return std::unique_ptr<pwm::IPwm>{device};
#else
    return std::unexpected(pwm::Error{"backend_unavailable", -1, -1});
#endif
}

std::expected<std::unique_ptr<camera::ICamera>, camera::CameraError>
    create_camera(const camera::Controls& controls) noexcept {
    auto created = camera::k230::Camera<4>::create(controls);
    if (!created)
        return std::unexpected(created.error());
    return std::unique_ptr<camera::ICamera>{std::move(created.value())};
}

template <typename Service>
int with_imu(
    const platform::ImuOptions& options, logging::Logger& log, Service&& service) noexcept {
    if (!options.enabled)
        return service(nullptr);
#ifdef DART_APP_IMU_K230
    /* Explicit bus/CS; remaining acquisition defaults are UNVALIDATED.
     * 1 kHz ODR/polling, +/-16 g, +/-2000 deg/s, zero configured gyro bias. */
    dart_iim42652_config_t config{};
    config.bus = options.bus;
    config.chip_select_pin = options.chip_select;
    config.bus_hz = 1'000'000;
    config.timeout_ms = 20;
    config.output_data_rate = 0x06;
    /* 2026-10-30 metrology: with RTC_MODE=1 this board produced 0 FIFO packets,
     * one DATA_RDY in 2 s and no frame change, i.e. the part stopped sampling,
     * while RTC_MODE=0 sampled at 1006 Hz. The 32.768 kHz CLKIN is therefore
     * not reaching pin 9, so the external clock must not be requested until it
     * is measured at the device pin. Set to 1 again only after that. */
    config.use_external_clock = 0;
    config.skip_device_id_check = options.ignore_device_id;
    imu::k230::Iim42652 device{config};
    const auto initialized = device.initialize();
    const auto& diagnostic = device.initialization_diagnostic();
    const char* level = initialized ? "INFO" : "ERROR";
    log.line(
        "level=%s module=imu op=identity id_check=%s id_read=%u expected_id=0x6f "
        "observed_id=0x%02x reset_id_read=%u reset_id=0x%02x\n",
        diagnostic.device_id_check_skipped ? "WARN" : level,
        diagnostic.device_id_check_skipped ? "skipped" : "enforced",
        diagnostic.device_id_read, diagnostic.observed_device_id,
        diagnostic.reset_device_id_read, diagnostic.reset_device_id);
    if (options.ignore_device_id) {
        constexpr char hex[] = "0123456789abcdef";
        char raw[29]{};
        for (unsigned i = 0; i < sizeof(diagnostic.raw_probe); ++i) {
            raw[2 * i] = hex[diagnostic.raw_probe[i] >> 4];
            raw[2 * i + 1] = hex[diagnostic.raw_probe[i] & 0x0f];
        }
        log.line(
            "level=%s module=imu op=raw_probe read=%u stage=%s register=0x1d length=14 "
            "bytes=%s verification=unverified accel_raw=%d,%d,%d gyro_raw=%d,%d,%d\n",
            level, diagnostic.raw_probe_read,
            dart_iim42652_stage_name(diagnostic.raw_probe_stage),
            diagnostic.raw_probe_read ? raw : "unavailable",
            diagnostic.last_accelerometer[0], diagnostic.last_accelerometer[1],
            diagnostic.last_accelerometer[2], diagnostic.last_gyroscope[0],
            diagnostic.last_gyroscope[1], diagnostic.last_gyroscope[2]);
    }
    log.line(
        "level=%s module=imu op=init_timing result=%d stage=%s "
        "start_ns=%llu first_access_ns=%llu reset_ns=%llu configured_ns=%llu "
        "enabled_ns=%llu first_read_ns=%llu ready_ns=%llu finished_ns=%llu polls=%u "
        "data_ready_polls=%u\n",
        level, diagnostic.result, dart_iim42652_stage_name(diagnostic.stage),
        static_cast<unsigned long long>(diagnostic.started_ns),
        static_cast<unsigned long long>(diagnostic.first_access_ns),
        static_cast<unsigned long long>(diagnostic.reset_ns),
        static_cast<unsigned long long>(diagnostic.configured_ns),
        static_cast<unsigned long long>(diagnostic.enabled_ns),
        static_cast<unsigned long long>(diagnostic.first_read_ns),
        static_cast<unsigned long long>(diagnostic.ready_ns),
        static_cast<unsigned long long>(diagnostic.finished_ns), diagnostic.readiness_polls,
        diagnostic.data_ready_polls);
    log.line(
        "level=%s module=imu op=config read=%u intf0=0x%02x intf1=0x%02x "
        "power=0x%02x gyro=0x%02x accel=0x%02x status=0x%02x status_seen=0x%02x "
        "sensor_config_read=%u sensor_config0=0x%02x external_clock_read=%u intf5=0x%02x "
        "accel_raw=%d,%d,%d gyro_raw=%d,%d,%d individual_read=%u gyro_individual=%d,%d,%d\n",
        level, diagnostic.registers_read, diagnostic.interface_config0, diagnostic.interface_config1,
        diagnostic.power_config, diagnostic.gyro_config, diagnostic.accel_config,
        diagnostic.interrupt_status, diagnostic.interrupt_status_seen,
        diagnostic.sensor_config_read, diagnostic.sensor_config0,
        diagnostic.external_clock_config_read, diagnostic.interface_config5,
        diagnostic.last_accelerometer[0], diagnostic.last_accelerometer[1],
        diagnostic.last_accelerometer[2], diagnostic.last_gyroscope[0],
        diagnostic.last_gyroscope[1], diagnostic.last_gyroscope[2],
        diagnostic.individual_gyroscope_read, diagnostic.individual_gyroscope[0],
        diagnostic.individual_gyroscope[1], diagnostic.individual_gyroscope[2]);
    if (!initialized) {
        log.line(
            "level=ERROR module=imu op=initialize code=%u bus=%d cs=%d "
            "id_read=%u expected_id=0x6f observed_id=0x%02x model=%s action=disable_imu\n",
            static_cast<unsigned>(initialized.error()), config.bus, config.chip_select_pin,
            diagnostic.device_id_read, diagnostic.observed_device_id,
            dart_iim42652_device_name(diagnostic.observed_device_id));
        return service(nullptr);
    }
    log.line(
        "level=INFO module=imu op=initialize model=%s device_id=0x%02x result=ok\n",
        dart_iim42652_device_name(diagnostic.observed_device_id), diagnostic.observed_device_id);
    if (options.writes == nullptr || options.writes->logging_enabled())
        storage::ensure_parent(options.log_path);
    imu::Session session{device};
    if (!session.start(options.log_path, 1000, true, imu::Session::kDefaultLogLimitBytes,
                       options.writes, options.sample_observer, options.sample_observer_context)) {
        log.line("level=ERROR module=imu op=start path=%s action=disable_imu\n", options.log_path);
        return service(nullptr);
    }
    log.line(
        "level=INFO module=imu op=start bus=%d cs=%d bus_hz=%u timeout_ms=%u "
        "odr=0x%02x accel_range=%u gyro_range=%u gyro_bias_rad_s=0,0,0 "
        "period_us=1000 initial_seed=stationary_identity parameters=UNVALIDATED "
        "log_limit_bytes=%llu path=%s\n",
        config.bus, config.chip_select_pin, config.bus_hz, config.timeout_ms,
        config.output_data_rate, config.accelerometer_range, config.gyroscope_range,
        static_cast<unsigned long long>(imu::Session::kDefaultLogLimitBytes), session.log_path());
    // Consumers must share this session's current()/seed()/reset() methods.
    const int result = service(&session);
    session.stop();
    const auto stats = session.stats();
    log.line(
        "level=%s module=imu op=stop samples=%llu read_errors=%llu written=%llu "
        "dropped=%llu io_errors=%llu log_bytes=%llu log_limit_reached=%d path=%s\n",
        stats.io_errors > 0 ? "ERROR" : "INFO", static_cast<unsigned long long>(stats.samples),
        static_cast<unsigned long long>(stats.read_errors),
        static_cast<unsigned long long>(stats.written),
        static_cast<unsigned long long>(stats.dropped),
        static_cast<unsigned long long>(stats.io_errors),
        static_cast<unsigned long long>(stats.log_bytes), stats.log_limit_reached,
        session.log_path());
    return result;
#else
    log.line("level=ERROR module=imu op=initialize code=backend_unavailable action=disable_imu\n");
    return service(nullptr);
#endif
}

} // namespace dart::board
