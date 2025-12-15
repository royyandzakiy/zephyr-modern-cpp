// zephyr_span_visit_concept.cpp
#include <iostream>
#include <vector>
#include <array>
#include <span>
#include <variant>
#include <string>
#include <optional>
#include <concepts>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(state_machine, LOG_LEVEL_INF);

// --- C++23 Feature Test Macros ---
#ifdef __has_include
#  if __has_include(<version>)
#    include <version>
#  endif
#  if __has_include(<ranges>)
#    include <ranges>
#    define HAS_RANGES 1
#  endif
#endif

using namespace std::chrono_literals;
constexpr auto STATE_UPDATE_INTERVAL = 2000ms;
constexpr auto LOG_INTERVAL = 5000ms;

// --- Time utilities for Zephyr ---
inline auto zephyr_sleep_for(std::chrono::milliseconds duration) {
    k_msleep(duration.count());
}

inline auto zephyr_get_task_name() -> const char* {
    return k_thread_name_get(k_current_get());
}

// --- Concepts & Constraints ---
template<typename T>
concept SensorType = requires(T t) {
    { t.read() } -> std::convertible_to<float>;
    { t.get_id() } -> std::convertible_to<int>;
};

template<typename T>
concept StateType = requires {
    requires std::is_enum_v<T> || std::is_class_v<T>;
};

// --- State Variant Definition ---
enum class StateId { IDLE, MONITORING, ALERT, CALIBRATING };
struct IdleState {};
struct MonitoringState { 
    float average_value; 
    int sample_count;
};
struct AlertState {
    std::string_view message;
    float threshold;
};
struct CalibratingState {
    float reference_value;
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
    auto read() const -> float { 
        // Simulate temperature reading
        return 23.5f + (sys_rand32_get() % 100) * 0.01f; 
    }
    auto get_id() const -> int { return 1; }
};

class HumiditySensor {
public:
    auto read() const -> float { 
        // Simulate humidity reading
        return 45.0f + (sys_rand32_get() % 100) * 0.02f; 
    }
    auto get_id() const -> int { return 2; }
};

class PressureSensor {
public:
    auto read() const -> float { 
        // Simulate pressure reading
        return 1013.25f + (sys_rand32_get() % 100) * 0.05f; 
    }
    auto get_id() const -> int { return 3; }
};

// --- State Machine with Variants & Visit ---
class StateMachine {
private:
    StateVariant current_state_{IdleState{}};
    StateId current_id_{StateId::IDLE};
    std::array<float, 10> sensor_buffer_{};
    size_t buffer_index_{0};

public:
    // --- Public accessor for buffer index ---
    [[nodiscard]] auto get_buffer_index() const -> size_t {
        return buffer_index_;
    }

    // --- Abbreviated Function Templates (C++20) ---
    auto transition_to(StateVariant new_state) -> void {
        current_state_ = new_state;
        current_id_ = static_cast<StateId>(current_state_.index());
    }

    // --- Using std::visit with variants ---
    // --- Using std::visit with variants ---
	auto get_state_info() const -> std::string {
		return std::visit([]<typename T>(const T& state) -> std::string {
			char buffer[128];
			if constexpr (std::is_same_v<T, IdleState>) {
				return "Idle - Waiting for commands";
			} else if constexpr (std::is_same_v<T, MonitoringState>) {
				snprintf(buffer, sizeof(buffer), "Monitoring - Avg: %.2f, Samples: %d",
						state.average_value, state.sample_count);
				return std::string(buffer);
			} else if constexpr (std::is_same_v<T, AlertState>) {
				snprintf(buffer, sizeof(buffer), "ALERT: %.*s (Threshold: %.1f)",
						static_cast<int>(state.message.length()),
						state.message.data(),
						state.threshold);
				return std::string(buffer);
			} else if constexpr (std::is_same_v<T, CalibratingState>) {
				snprintf(buffer, sizeof(buffer), "Calibrating - Ref: %.2f, Step: %d",
						state.reference_value, state.calibration_step);
				return std::string(buffer);
			}
		}, current_state_);
	}

    // --- Process sensors using span ---
    template<SensorType... Sensors>
    auto process_sensors(Sensors&&... sensors) -> void {
        // Create span of sensor readings
        std::array<float, sizeof...(sensors)> readings{sensors.read()...};
        std::span<const float> readings_span{readings};
        
        // Update buffer with first reading
        if (!readings_span.empty()) {
            sensor_buffer_[buffer_index_ % sensor_buffer_.size()] = readings_span[0];
            buffer_index_++;
        }

        // State transition logic
        std::visit([this, readings_span](auto& state) {
            using T = std::decay_t<decltype(state)>;
            
            if constexpr (std::is_same_v<T, IdleState>) {
                if (!readings_span.empty() && readings_span[0] > 20.0f) {
                    transition_to(MonitoringState{readings_span[0], 1});
                }
            } else if constexpr (std::is_same_v<T, MonitoringState>) {
                state.sample_count++;
                
                // Calculate average using span
                float sum = 0;
                size_t count = std::min(buffer_index_, sensor_buffer_.size());
                for (size_t i = 0; i < count; ++i) {
                    sum += sensor_buffer_[i];
                }
                state.average_value = (count > 0) ? sum / count : 0.0f;
                
                if (state.average_value > 30.0f) {
                    transition_to(AlertState{"Temperature High", 30.0f});
                }
            } else if constexpr (std::is_same_v<T, AlertState>) {
                if (!readings_span.empty() && readings_span[0] < 25.0f) {
                    transition_to(CalibratingState{22.5f, 1});
                }
            } else if constexpr (std::is_same_v<T, CalibratingState>) {
                state.calibration_step++;
                if (state.calibration_step > 5) {
                    transition_to(IdleState{});
                }
            }
        }, current_state_);
    }

    // --- Get buffer statistics using span ---
    auto get_buffer_stats() const -> std::tuple<float, float> {
        if (buffer_index_ == 0) return {0.0f, 0.0f};
        
        std::span<const float> active_buffer{
            sensor_buffer_.data(), 
            std::min(buffer_index_, sensor_buffer_.size())
        };
        
        float min_val = std::numeric_limits<float>::max();
        float max_val = std::numeric_limits<float>::lowest();
        
        for (auto val : active_buffer) {
            min_val = std::min(min_val, val);
            max_val = std::max(max_val, val);
        }
        
        return {min_val, max_val};
    }

    auto get_current_state_id() const -> StateId { return current_id_; }
};

// --- Thread-safe State Machine Manager ---
class StateMachineManager {
private:
    StateMachine state_machine_;
    TemperatureSensor temp_sensor_;
    HumiditySensor humidity_sensor_;
    PressureSensor pressure_sensor_;
    
public:
    auto update() -> void {
        // Process all sensors
        state_machine_.process_sensors(temp_sensor_, humidity_sensor_, pressure_sensor_);
        
        // Get buffer stats using structured binding
        auto [min_val, max_val] = state_machine_.get_buffer_stats();
        
        // Log state with buffer info
        LOG_INF("State: %s | Buffer: %zu samples | Range: [%.1f, %.1f]",
            state_machine_.get_state_info().c_str(),
            std::min(state_machine_.get_buffer_index(), 
                    static_cast<size_t>(10)),
            min_val, max_val);
    }
    
    [[nodiscard]] auto get_state_id() const -> StateId {
        return state_machine_.get_current_state_id();
    }
};

// --- Thread Functions with C++23 Features ---
void state_monitor_thread([[maybe_unused]] void* p1, [[maybe_unused]] void* p2, [[maybe_unused]] void* p3) {
    auto thread_id = reinterpret_cast<int>(p1);
    StateMachineManager manager;
    const char* task_name = zephyr_get_task_name();
    
    while (true) {
        // if with initializer
        if (auto state = manager.get_state_id(); state == StateId::ALERT) {
            LOG_WRN("Thread %d: CRITICAL ALERT STATE", thread_id);
        }
        
        manager.update();
        zephyr_sleep_for(STATE_UPDATE_INTERVAL);
    }
}

void sensor_processor_thread([[maybe_unused]] void* p1, [[maybe_unused]] void* p2, [[maybe_unused]] void* p3) {
    const char* task_name = zephyr_get_task_name();
    std::vector<StateMachineManager> managers(3);
    
    // Range-based for with init - using the manager
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
    const char* main_task_name = zephyr_get_task_name();
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