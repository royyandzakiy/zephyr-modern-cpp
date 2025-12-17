// main.cpp
#include <array>
#include <chrono>
#include <concepts>
#include <cstdio>
#include <limits>
#include <ranges>
#include <span>
#include <string_view>
#include <tuple>
#include <variant>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

LOG_MODULE_REGISTER(state_machine, LOG_LEVEL_INF);

// --- C++23 Feature Test Macros ---
#ifdef __has_include
#if __has_include(<ranges>)
#include <ranges>
#define HAS_RANGES 1
#endif
#endif

using namespace std::chrono_literals;
using namespace std::string_view_literals;
constexpr auto STATE_UPDATE_INTERVAL = 2000ms;
constexpr auto LOG_INTERVAL = 5000ms;

constexpr int32_t TEMP_THRESHOLD_START_MONITORING = 2000; // 20.00°C
constexpr int32_t TEMP_THRESHOLD_ALERT = 3000;			  // 30.00°C
constexpr int32_t TEMP_THRESHOLD_RECOVERY = 2500;		  // 25.00°C
constexpr int32_t CALIBRATION_REFERENCE_TEMP = 2250;	  // 22.50°C
constexpr int MAX_CALIBRATION_STEPS = 5;

// --- Time utilities for Zephyr ---
inline auto zephyr_sleep_for(std::chrono::milliseconds duration) {
	k_msleep(duration.count());
}

inline auto zephyr_get_task_name() -> std::string_view {
	const char *name = k_thread_name_get(k_current_get());
	return name ? std::string_view{name} : "unnamed"sv;
}

// ---------------- Fixed-point helpers ----------------
constexpr int SCALE = 100;

constexpr int ipart(int v) {
	return v / SCALE;
}
constexpr int fpart(int v) {
	return v < 0 ? -(v % SCALE) : (v % SCALE);
}

// --- State Variant Definition ---
enum class StateId {
	IDLE,
	MONITORING,
	ALERT,
	CALIBRATING
};
struct IdleState {};
struct MonitoringState {
	int32_t average_temp_value;
	int sample_count;
};

struct AlertState {
	std::string_view message;
	int32_t threshold_temp_value;
};

struct CalibratingState {
	int32_t reference_temp_value;
	int calibration_step;
};

using StateVariant = std::variant<IdleState, MonitoringState, AlertState, CalibratingState>;

// --- Overloaded Helper ---

template <typename... T> struct Overloaded : T... {
	using T::operator()...;
};
// deduction guide if using C++17. in C++20 solved by Class Template Argument Deduction (CTAD) for aggregates
// template <typename... T> Overloaded(T...) -> Overloaded<T...>; // no need

// --- Concepts & Constraints ---
template <typename T>
concept SensorType = requires(T t) {
	{ t.read() } -> std::convertible_to<int32_t>;
	{ t.get_id() } -> std::convertible_to<int>;
};

// --- Sensor Concepts Implementation ---
class TemperatureSensor {
  public:
	auto read() const -> int32_t {
		return 2350 + (sys_rand32_get() % 100); // 23.50 – 24.49
	}
	auto get_id() const -> int {
		return 1;
	}
};

class HumiditySensor {
  public:
	auto read() const -> int32_t {
		return 4500 + (sys_rand32_get() % 200); // 45.00 – 46.99
	}
	auto get_id() const -> int {
		return 2;
	}
};

class PressureSensor {
  public:
	auto read() const -> int32_t {
		return 101325 + (sys_rand32_get() % 500); // 1013.25+
	}
	auto get_id() const -> int {
		return 3;
	}
};

// --- State Machine with Variants & Visit ---
class StateMachine {
  private:
	StateVariant current_state_{IdleState{}};
	StateId current_id_{StateId::IDLE};

	std::array<int32_t, 10> temp_data_circbuf_{};
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
		static std::array<char, 128> info_desc{};
		constexpr size_t size = info_desc.size();

		// clang-format off
		return std::visit(Overloaded{
			[](const IdleState&) -> std::string_view {
				return "Status: Idle - Awaiting for sensor trigger"sv;
			},
			[](const MonitoringState& s) -> std::string_view {
				int len = snprintf(info_desc.data(), size, "Status: Monitoring [Avg: %d.%02d°C | Samples: %d]",
									ipart(s.average_temp_value), fpart(s.average_temp_value), s.sample_count);
				return { info_desc.data(), static_cast<size_t>(len) };
			},
			[](const AlertState& s) -> std::string_view {
				int len = snprintf(info_desc.data(), size, "Status: ALERT [%.*s | Threshold: %d.%02d°C]", 
								static_cast<int>(s.message.length()), s.message.data(),
								ipart(s.threshold_temp_value), fpart(s.threshold_temp_value));
				return {info_desc.data(), static_cast<size_t>(len)};
			},
			[](const CalibratingState& s) -> std::string_view {
				int len = snprintf(info_desc.data(), size, "Status: Calibrating [Ref: %d.%02d°C | Step: %d/5]", 
								ipart(s.reference_temp_value), fpart(s.reference_temp_value), s.calibration_step);
				return {info_desc.data(), static_cast<size_t>(len)};
			}
		}, current_state_);
		// clang-format on
	}

	template <SensorType... Sensors> auto process_sensors(Sensors &&...sensors) -> void {
		const std::array current_readings{sensors.read()...};
		const std::span readings_view{current_readings};

		// Extract primary sensor data for clear intent
		const int32_t current_temp = readings_view[0];
		const size_t history_count = std::min(buffer_index_, temp_data_circbuf_.size());

		// Update circular buffer
		temp_data_circbuf_[buffer_index_ % temp_data_circbuf_.size()] = current_temp;
		buffer_index_++;

		std::visit(
			[this, current_temp, history_count]<typename T>(T &state) {
				if constexpr (std::is_same_v<T, IdleState>) {
					if (current_temp > TEMP_THRESHOLD_START_MONITORING) {
						transition_to(MonitoringState{current_temp, 1});
					}
				}

				else if constexpr (std::is_same_v<T, MonitoringState>) {
					state.sample_count++;

					int64_t sum = 0;
					for (size_t i = 0; i < history_count; ++i) {
						sum += temp_data_circbuf_[i];
					}

					state.average_temp_value = static_cast<int32_t>(sum / history_count);

					if (state.average_temp_value > TEMP_THRESHOLD_ALERT) {
						transition_to(AlertState{"Temperature High"sv, TEMP_THRESHOLD_ALERT});
					}
				}

				else if constexpr (std::is_same_v<T, AlertState>) {
					if (current_temp < TEMP_THRESHOLD_RECOVERY) {
						transition_to(CalibratingState{CALIBRATION_REFERENCE_TEMP, 1});
					}
				}

				else if constexpr (std::is_same_v<T, CalibratingState>) {
					if (++state.calibration_step > MAX_CALIBRATION_STEPS) {
						transition_to(IdleState{});
					}
				}
			},
			current_state_);
	}

	auto get_history_range() const -> std::tuple<int32_t, int32_t> {
		if (buffer_index_ == 0) {
			return {0, 0};
		}

		const size_t history_count = std::min(buffer_index_, temp_data_circbuf_.size());

		int32_t min_temp = temp_data_circbuf_[0];
		int32_t max_temp = temp_data_circbuf_[0];

		for (size_t i = 1; i < history_count; ++i) {
			min_temp = std::min(min_temp, temp_data_circbuf_[i]);
			max_temp = std::max(max_temp, temp_data_circbuf_[i]);
		}

		return {min_temp, max_temp};
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

		auto [min_temp, max_temp] = sm_.get_history_range();
		auto info = sm_.get_state_info();

		LOG_INF("State: %.*s | Buffer: %zu | Range: [%d.%02d, %d.%02d]", (int)info.length(), info.data(),
				sm_.get_buffer_index(), ipart(min_temp), fpart(min_temp), ipart(max_temp), fpart(max_temp));
	}

	auto get_state_id() const -> StateId {
		return sm_.get_current_state_id();
	}
};
// --- Thread Functions ---
void state_monitor_thread([[maybe_unused]] void *p1, [[maybe_unused]] void *p2, [[maybe_unused]] void *p3) {
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

void sensor_processor_thread([[maybe_unused]] void *p1, [[maybe_unused]] void *p2, [[maybe_unused]] void *p3) {
	// Changed from std::vector to std::array (static allocation)
	std::array<StateMachineManager, 3> managers{};

	// Range-based for with init
	for (size_t i = 0; auto &manager : managers) {
		LOG_INF("Initialized manager %zu", i++);
		manager.update();
	}

	while (true) {
		// Process each manager
		for (auto &manager : managers) {
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
void main_thread_entry(void *p1, void *p2, void *p3) {
	LOG_INF("Starting C++23 State Machine Example on Zephyr");

	// Create threads
	k_thread_create(&state_mon_thread1, state_mon_stack1, K_THREAD_STACK_SIZEOF(state_mon_stack1), state_monitor_thread,
					reinterpret_cast<void *>(1), nullptr, nullptr, K_PRIO_COOP(5), 0, K_NO_WAIT);
	k_thread_name_set(&state_mon_thread1, "StateMon1");

	k_thread_create(&state_mon_thread2, state_mon_stack2, K_THREAD_STACK_SIZEOF(state_mon_stack2), state_monitor_thread,
					reinterpret_cast<void *>(2), nullptr, nullptr, K_PRIO_COOP(5), 0, K_NO_WAIT);
	k_thread_name_set(&state_mon_thread2, "StateMon2");

	k_thread_create(&sensor_proc_thread, sensor_proc_stack, K_THREAD_STACK_SIZEOF(sensor_proc_stack),
					sensor_processor_thread, nullptr, nullptr, nullptr, K_PRIO_COOP(6), 0, K_NO_WAIT);
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
	k_thread_create(&main_thread, main_stack, K_THREAD_STACK_SIZEOF(main_stack), main_thread_entry, nullptr, nullptr,
					nullptr, K_PRIO_COOP(4), 0, K_NO_WAIT);
	k_thread_name_set(&main_thread, "Main");

	// Let the scheduler run
	k_thread_join(&main_thread, K_FOREVER);

	return 0;
}