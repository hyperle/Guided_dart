#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <expected>
#include <functional>
#include <map>
#include <memory>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <vector>

namespace dart::executor {

enum class Error {
    invalid_state,
    invalid_spec,
    output_failed,
    invalid_factory,
    duplicate_type,
    unknow_type,
    creation_fail
};


inline  std::string_view discribe(Error error) noexcept {
    switch (error) {
        case executor::Error::invalid_state:   return "invlaid lifecycle state";
        case executor::Error::invalid_spec:    return "invlaid component configuration";
        case executor::Error::output_failed:   return "output failed";
        case executor::Error::invalid_factory: return "invlaid component registration";
        case executor::Error::duplicate_type:  return "duplicate component type";
        case executor::Error::unknow_type:     return "unknow component type";
        case executor::Error::creation_fail:   return "component factory returned null";
    }
    return "unknown error";
}

template<typename T>
class AtomicValue final {
    static_assert(std::is_trivially_copyable_v<T>);
    static_assert(std::atomic<T>::is_always_lock_free);

public:
    explicit AtomicValue(T initial = {}) noexcept
        : value_(initial) {}

    T read() const noexcept {
        return value_.load(std::memory_order_relaxed);
    }

    void write(T value) noexcept {
        value_.store(value, std::memory_order_relaxed);
    }

private:
    std::atomic<T> value_;
};

// Ports borrow a channel; its lifetime must cover every component using it.
// Reads and writes retain the channel's synchronization and buffering semantics.
// Input<T> uses the channel's read result type, including snapshot metadata.
template<typename T>
class Input final {
public:
    template<typename Channel>
        requires (!std::same_as<std::remove_cv_t<Channel>, Input>)
            && requires(const Channel& channel) {
                { channel.read() } -> std::same_as<T>;
            }
    explicit Input(Channel& channel) noexcept
        : channel_(std::addressof(channel))
        , read_([](const void* source) -> T {
            return static_cast<const Channel*>(source)->read();
        }) {}

    T read() const {
        return read_(channel_);
    }

private:
    const void* channel_;
    T (*read_)(const void*);
};

template<typename T>
class Output final {
public:
    template<typename Channel>
        requires (!std::is_const_v<Channel>)
            && requires(Channel& channel, const T& value) {
                { channel.write(value) } -> std::same_as<void>;
            }
    explicit Output(Channel& channel) noexcept
        : channel_(std::addressof(channel))
        , write_([](void* destination, const T& value) {
            static_cast<Channel*>(destination)->write(value);
        }) {}

    void publish(const T& value) const {
        write_(channel_, value);
    }

private:
    void* channel_;
    void (*write_)(void*, const T&);
};

class Component {
public:
    struct Spec {
        std::string name;
        std::string type;
        double frequence_hz = 0.0;
        std::map<std::string, double, std::less<>> parameter;

        std::expected<void, Error> validate() const noexcept {
            if( name.empty() || type.empty() ||
                !std::isfinite(frequence_hz) || frequence_hz < 1.0 || frequence_hz > 10000.0) {
                return std::unexpected(Error::invalid_spec);
            }

            for (const auto& [key, value] : parameter) {
                if (key.empty() || !std::isfinite(value)) {
                    return std::unexpected(Error::invalid_spec);
                }
            }

            return {};
        }
    };

    struct Statistics {
        std::uint64_t ticks = 0;
        std::uint64_t skipped_periods = 0;
        std::chrono::nanoseconds max_tick_time{};
    };

    explicit Component(const Spec& spec)
        : name_(spec.name) {

        if (spec.validate()) {
            period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>{1.0 / spec.frequence_hz});
        } 
    }

    virtual ~Component() = default;

    Component(const Component&) = delete;

    Component& operator=(const Component&) = delete;

    std::expected<void, Error> initialze() noexcept {
        if (initialized_ ||has_run_) {
            return std::unexpected(Error::invalid_state);
        }

        if (period_ <= std::chrono::steady_clock::duration::zero()) {
            return std::unexpected(Error::invalid_spec);
        }

        const auto result = initializing();
        if (!result) {
            return result;
        }

        initialized_ = true;

        return {};
    }

    std::expected<void, Error> run(const std::stop_token& stop) noexcept {
        if (!initialized_ || has_run_) {
            return std::unexpected(Error::invalid_state);
        }

        has_run_ = true;
        auto next = std::chrono::steady_clock::now();

        while (!stop.stop_requested()) {
            const auto started = std::chrono::steady_clock::now();
            const auto result = tick();
            const auto finished = std::chrono::steady_clock::now();

            ++statistics_.ticks;
            statistics_.max_tick_time = std::max(statistics_.max_tick_time, 
                                                 std::chrono::duration_cast<std::chrono::nanoseconds>(finished - started));
            if (!result) {
                return result;
            }

            next += period_;

            if (finished > next) {
                const auto skipped = (finished - next) /period_ + 1;
                next += period_ * skipped;
                statistics_.skipped_periods += static_cast<std::uint64_t>(skipped);
            }

            if (!stop.stop_requested()) {
                std::this_thread::sleep_until(next);
            }
        }

        return {};
    }

    std::expected<void, Error> shutdown() noexcept {
        if (!initialized_) {
            return {};
        }
        initialized_ = false;
        return shutingdowm();
    }

    std::string_view name() const noexcept {
        return name_;
    }

    const Statistics& statistics() const noexcept {
        return statistics_;
    }

protected:
    virtual std::expected<void, Error> initializing() {return {};}
    virtual std::expected<void, Error> capture_inputs() noexcept = 0;
    virtual std::expected<void, Error> update() noexcept = 0;
    virtual std::expected<void, Error> publish_outputs() noexcept = 0;
    virtual std::expected<void, Error> shutingdowm() noexcept {return {};};

private:
    std::expected<void, Error> tick() noexcept {
        auto result = capture_inputs();
        if (!result) {
            return result;
        }

        result = update();
        if (!result) {
            return result;
        }

        return publish_outputs();
    }

    std::string name_;
    std::chrono::steady_clock::duration period_{};
    Statistics statistics_;
    bool initialized_ = false;
    bool has_run_ = false;
};

class Registry final {
public:
    std::expected<void, Error> add(const std::string& type, std::function<std::expected<std::unique_ptr<Component>, Error>(const Component::Spec&)> factory) {
        if(type.empty() || !factory) {
            return std::unexpected(Error::invalid_factory);
        }

        if (!factories_.try_emplace(type, factory).second) {
            return std::unexpected(Error::duplicate_type);
        }

        return {};
    }

    std::expected<std::unique_ptr<Component>, Error> create(const Component::Spec& spec) const {
        if (const auto result = spec.validate(); !result) {
            return std::unexpected(Error::invalid_factory);
        }

        const auto entry = factories_.find(spec.type);
        if (entry == factories_.end()) {
            return std::unexpected(Error::unknow_type);
        }

        auto result = entry->second(spec);
        if (!result) {
            return result;
        }

        if (!*result) {
            return std::unexpected(Error::creation_fail);
        }

        return  result;
    }

private:
    std::map<std::string, std::function<std::expected<std::unique_ptr<Component>, Error>(const Component::Spec&)>, std::less<>> factories_;
};

struct Configuration {
    std::chrono::milliseconds run_for{250};
    std::vector<Component::Spec> components;
};


} // namespace dart::executor
