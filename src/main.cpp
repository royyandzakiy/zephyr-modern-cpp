// zephyr_span_visit_concept.cpp
#include <tuple>
#include <array>
#include <span>
#include <variant>
#include <string_view>
#include <concepts>
#include <limits>
#include <cstdio>
#include <chrono>
#include <ranges>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(state_machine, LOG_LEVEL_INF);

// --- C++23 Feature Test Macros ---
#ifdef __has_include
#  if __has_include(<ranges>)
#    include <ranges>
#    define HAS_RANGES 1
#  endif
#endif

using namespace std::chrono_literals;
using namespace std::string_view_literals;
constexpr auto STATE_UPDATE_INTERVAL = 2000ms;
constexpr auto LOG_INTERVAL = 5000ms;

// --- Time utilities for Zephyr ---
inline auto zephyr_sleep_for(std::chrono::milliseconds duration) {
    k_msleep(duration.count());
}

inline auto zephyr_get_task_name() -> std::string_view {
    const char* name = k_thread_name_get(k_current_get());
    return name ? std::string_view{name} : "unnamed"sv;
}

// ---------------- Fixed-point helpers ----------------
constexpr int SCALE = 100;

constexpr int ipart(int v) { return v / SCALE; }
constexpr int fpart(int v) { return v < 0 ? -(v % SCALE) : (v % SCALE); }

// --- Concepts & Constraints ---
template<typename T>
concept SensorType = requires(T t) {
    { t.read() } -> std::convertible_to<int32_t>;
    { t.get_id() } -> std::convertible_to<int>;
};

// --- State Variant Definition ---
enum class StateId { IDLE, MONITORING, ALERT, CALIBRATING };
struct IdleState {};
struct MonitoringState {
    int32_t average_centi;
    int sample_count;
};

struct AlertState {
    std::string_view message;
    int32_t threshold_centi;
};

struct CalibratingState {
    int32_t reference_centi;
    int calibration_step;
};

using StateVariant = std::variant<
    IdleState, 
    MonitoringState, 
    AlertState, 
    CalibratingState
>;

// --- Sensor Concepts Implementation ---
class TemperatureSensor {
public:
    auto read() const -> int32_t {
        return 2350 + (sys_rand32_get() % 100);  // 23.50 – 24.49
    }
    auto get_id() const -> int { return 1; }
};

class HumiditySensor {
public:
    auto read() const -> int32_t {
        return 4500 + (sys_rand32_get() % 200);  // 45.00 – 46.99
    }
    auto get_id() const -> int { return 2; }
};

class PressureSensor {
public:
    auto read() const -> int32_t {
        return 101325 + (sys_rand32_get() % 500); // 1013.25+
    }
    auto get_id() const -> int { return 3; }
};

// --- State Machine with Variants & Visit ---
class StateMachine {
private:
    StateVariant current_state_{IdleState{}};
    StateId current_id_{StateId::IDLE};

    std::array<int32_t, 10> sensor_buffer_{};
    size_t buffer_index_{0};

public:
    [[nodiscard]] auto get_buffer_index() const -> size_t {
        return buffer_index_;
    }

    auto transition_to(StateVariant s) -> void {
        current_state_ = s;
        current_id_ = static_cast<StateId>(current_state_.index());
    }

    auto get_state_info() const -> std::string_view {
        return std::visit([]<typename T>(const T& state) -> std::string_view {
            static std::array<char, 128> buf{};

            if constexpr (std::is_same_v<T, IdleState>) {
                return "Idle - Waiting for commands"sv;
            }
            else if constexpr (std::is_same_v<T, MonitoringState>) {
                int len = snprintf(buf.data(), buf.size(),
                    "Monitoring - Avg: %d.%02d, Samples: %d",
                    ipart(state.average_centi),
                    fpart(state.average_centi),
                    state.sample_count);
                return {buf.data(), (size_t)len};
            }
            else if constexpr (std::is_same_v<T, AlertState>) {
                int len = snprintf(buf.data(), buf.size(),
                    "ALERT: %.*s (Threshold: %d.%02d)",
                    (int)state.message.length(),
                    state.message.data(),
                    ipart(state.threshold_centi),
                    fpart(state.threshold_centi));
                return {buf.data(), (size_t)len};
            }
            else if constexpr (std::is_same_v<T, CalibratingState>) {
                int len = snprintf(buf.data(), buf.size(),
                    "Calibrating - Ref: %d.%02d, Step: %d",
                    ipart(state.reference_centi),
                    fpart(state.reference_centi),
                    state.calibration_step);
                return {buf.data(), (size_t)len};
            }
            return "Unknown"sv;
        }, current_state_);
    }

    template<SensorType... Sensors>
    auto process_sensors(Sensors&&... sensors) -> void {
        std::array<int32_t, sizeof...(sensors)> readings{sensors.read()...};
        std::span<const int32_t> span{readings};

        if (!span.empty()) {
            sensor_buffer_[buffer_index_ % sensor_buffer_.size()] = span[0];
            buffer_index_++;
        }

        std::visit([this, span](auto& state) {
            using T = std::decay_t<decltype(state)>;

            if constexpr (std::is_same_v<T, IdleState>) {
                if (!span.empty() && span[0] > 2000) {
                    transition_to(MonitoringState{span[0], 1});
                }
            }
            else if constexpr (std::is_same_v<T, MonitoringState>) {
                state.sample_count++;

                size_t n = std::min(buffer_index_, sensor_buffer_.size());
                int64_t sum = 0;
                for (size_t i = 0; i < n; ++i)
                    sum += sensor_buffer_[i];

                state.average_centi = (n > 0) ? (sum / (int64_t)n) : 0;

                if (state.average_centi > 3000) {
                    transition_to(AlertState{"Temperature High"sv, 3000});
                }
            }
            else if constexpr (std::is_same_v<T, AlertState>) {
                if (!span.empty() && span[0] < 2500) {
                    transition_to(CalibratingState{2250, 1});
                }
            }
            else if constexpr (std::is_same_v<T, CalibratingState>) {
                if (++state.calibration_step > 5) {
                    transition_to(IdleState{});
                }
            }
        }, current_state_);
    }

    auto get_buffer_stats() const -> std::tuple<int32_t, int32_t> {
        if (buffer_index_ == 0)
            return {0, 0};

        size_t n = std::min(buffer_index_, sensor_buffer_.size());
        int32_t min_v = sensor_buffer_[0];
        int32_t max_v = sensor_buffer_[0];

        for (size_t i = 1; i < n; ++i) {
            min_v = std::min(min_v, sensor_buffer_[i]);
            max_v = std::max(max_v, sensor_buffer_[i]);
        }
        return {min_v, max_v};
    }

    auto get_current_state_id() const -> StateId {
        return current_id_;
    }
};

// --- Thread-safe State Machine Manager ---
class StateMachineManager {
private:
    StateMachine sm_;
    TemperatureSensor t_;
    HumiditySensor h_;
    PressureSensor p_;

public:
    auto update() -> void {
        sm_.process_sensors(t_, h_, p_);

        auto [min_v, max_v] = sm_.get_buffer_stats();
        auto info = sm_.get_state_info();

        LOG_INF("State: %.*s | Buffer: %zu | Range: [%d.%02d, %d.%02d]",
            (int)info.length(), info.data(),
            sm_.get_buffer_index(),
            ipart(min_v), fpart(min_v),
            ipart(max_v), fpart(max_v));
    }

    auto get_state_id() const -> StateId {
        return sm_.get_current_state_id();
    }
};
// --- Thread Functions ---
void state_monitor_thread([[maybe_unused]] void* p1, [[maybe_unused]] void* p2, [[maybe_unused]] void* p3) {
    const auto thread_id = reinterpret_cast<intptr_t>(p1);
    StateMachineManager manager;
    
    while (true) {
        // if with initializer
        if (auto state = manager.get_state_id(); state == StateId::ALERT) {
            LOG_WRN("Thread %ld: CRITICAL ALERT STATE", static_cast<long>(thread_id));
        }
        
        manager.update();
        zephyr_sleep_for(STATE_UPDATE_INTERVAL);
    }
}

void sensor_processor_thread([[maybe_unused]] void* p1, [[maybe_unused]] void* p2, [[maybe_unused]] void* p3) {
    // Changed from std::vector to std::array (static allocation)
    std::array<StateMachineManager, 3> managers{};
    
    // Range-based for with init
    for (size_t i = 0; auto& manager : managers) {
        LOG_INF("Initialized manager %zu", i++);
        manager.update();
    }
    
    while (true) {
        // Process each manager
        for (auto& manager : managers) {
            manager.update();
        }
        
        zephyr_sleep_for(1000ms);
    }
}

// --- Thread stack definitions ---
K_THREAD_STACK_DEFINE(state_mon_stack1, 4096);
K_THREAD_STACK_DEFINE(state_mon_stack2, 4096);
K_THREAD_STACK_DEFINE(sensor_proc_stack, 4096);
K_THREAD_STACK_DEFINE(main_stack, 2048);

// --- Thread structures ---
struct k_thread state_mon_thread1;
struct k_thread state_mon_thread2;
struct k_thread sensor_proc_thread;
struct k_thread main_thread;

// --- Main Application Thread ---
void main_thread_entry(void* p1, void* p2, void* p3) {
    LOG_INF("Starting C++23 State Machine Example on Zephyr");
    
    // Create threads
    k_thread_create(&state_mon_thread1, state_mon_stack1,
                    K_THREAD_STACK_SIZEOF(state_mon_stack1),
                    state_monitor_thread,
                    reinterpret_cast<void*>(1), nullptr, nullptr,
                    K_PRIO_COOP(5), 0, K_NO_WAIT);
    k_thread_name_set(&state_mon_thread1, "StateMon1");
    
    k_thread_create(&state_mon_thread2, state_mon_stack2,
                    K_THREAD_STACK_SIZEOF(state_mon_stack2),
                    state_monitor_thread,
                    reinterpret_cast<void*>(2), nullptr, nullptr,
                    K_PRIO_COOP(5), 0, K_NO_WAIT);
    k_thread_name_set(&state_mon_thread2, "StateMon2");
    
    k_thread_create(&sensor_proc_thread, sensor_proc_stack,
                    K_THREAD_STACK_SIZEOF(sensor_proc_stack),
                    sensor_processor_thread,
                    nullptr, nullptr, nullptr,
                    K_PRIO_COOP(6), 0, K_NO_WAIT);
    k_thread_name_set(&sensor_proc_thread, "SensorProc");
    
    // Main loop
    int cycle = 0;
    
    while (true) {
        LOG_INF("Main task cycle %d", ++cycle);
        zephyr_sleep_for(LOG_INTERVAL);
    }
}

// --- Main entry point ---
extern "C" int main(void) {
    // Create main thread
    k_thread_create(&main_thread, main_stack,
                    K_THREAD_STACK_SIZEOF(main_stack),
                    main_thread_entry,
                    nullptr, nullptr, nullptr,
                    K_PRIO_COOP(4), 0, K_NO_WAIT);
    k_thread_name_set(&main_thread, "Main");
    
    // Let the scheduler run
    k_thread_join(&main_thread, K_FOREVER);
    
    return 0;
}