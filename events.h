
#pragma once

#include "px4_msgs/msg/arming_check_reply.hpp"


namespace events
{

using EventType = px4_msgs::msg::Event;

enum class LogLevel : uint8_t {
	Emergency = 0,
	Alert = 1,
	Critical = 2,
	Error = 3,
	Warning = 4,
	Notice = 5,
	Info = 6,
	Debug = 7,
	Protocol = 8,
	Disabled = 9,

	Count
};

enum class LogLevelInternal : uint8_t {
	Emergency = 0,
	Alert = 1,
	Critical = 2,
	Error = 3,
	Warning = 4,
	Notice = 5,
	Info = 6,
	Debug = 7,
	Protocol = 8,
	Disabled = 9,

	Count
};

using Log = LogLevel;
using LogInternal = LogLevelInternal;

struct LogLevels {
	LogLevels() {}
	LogLevels(Log external_level) : external(external_level), internal((LogInternal)external_level) {}
	LogLevels(Log external_level, LogInternal internal_level)
		: external(external_level), internal(internal_level) {}

	Log external{Log::Info};
	LogInternal internal{LogInternal::Info};
};

namespace util
{

// source: https://gist.github.com/ruby0x1/81308642d0325fd386237cfa3b44785c
constexpr uint32_t val_32_const = 0x811c9dc5;
constexpr uint32_t prime_32_const = 0x1000193;
constexpr uint64_t val_64_const = 0xcbf29ce484222325;
constexpr uint64_t prime_64_const = 0x100000001b3;
inline constexpr uint32_t hash_32_fnv1a_const(const char *const str, const uint32_t value = val_32_const) noexcept
{
	return (str[0] == '\0') ? value : hash_32_fnv1a_const(&str[1], (value ^ uint32_t(str[0])) * prime_32_const);
}

template<typename T>
inline constexpr void fillEventArguments(uint8_t *buf, T arg)
{
	// This assumes we're on little-endian
	memcpy(buf, &arg, sizeof(T));
}

template<typename T, typename... Args>
inline constexpr void fillEventArguments(uint8_t *buf, T arg, Args... args)
{
	fillEventArguments(buf, arg);
	fillEventArguments(buf + sizeof(T), args...);
}

constexpr unsigned sizeofArguments() { return 0; }

template <typename T, typename... Args>
constexpr unsigned sizeofArguments(const T &t, const Args &... args)
{
	return sizeof(T) + sizeofArguments(args...);
}

} // namespace util

/**
 * Generate event ID from an event name
 */
template<size_t N>
constexpr uint32_t ID(const char (&name)[N])
{
	// Note: the generated ID must match with the python generator under Tools/px4events
	uint32_t component_id = 1u << 24; // autopilot component
	return (0xffffff & util::hash_32_fnv1a_const(name)) | component_id;
}

namespace px4 // component id: 1
{
namespace enums
{

enum class health_component_t : uint32_t {
	none = 1, ///< None
	absolute_pressure = 2, ///< Absolute pressure
	differential_pressure = 4, ///< Differential pressure
	gps = 8, ///< GPS
	optical_flow = 16, ///< Optical flow
	vision_position = 32, ///< Vision position estimate
	distance_sensor = 64, ///< Distance sensor
	manual_control_input = 128, ///< RC or virtual joystick input
	motors_escs = 256, ///< Motors/ESCs
	utm = 512, ///< UTM
	logging = 1024, ///< Logging
	battery = 2048, ///< Battery
	communication_links = 4096, ///< Communication links
	rate_controller = 8192, ///< Rate controller
	attitude_controller = 16384, ///< Attitude controller
	position_controller = 32768, ///< Position controller
	attitude_estimate = 65536, ///< Attitude estimate
	local_position_estimate = 131072, ///< Local position estimate
	mission = 262144, ///< Mission
	avoidance = 524288, ///< Avoidance
	system = 1048576, ///< System
	camera = 2097152, ///< Camera
	gimbal = 4194304, ///< Gimbal
	payload = 8388608, ///< Payload
	global_position_estimate = 16777216, ///< Global position estimate
	storage = 33554432, ///< Storage
	parachute = 67108864, ///< Parachute
	imu = 134217728, ///< IMU

	_max = 134217728
};
} // namespace enums
} // namespace px4


} // namespace events




using health_component_t = events::px4::enums::health_component_t;

class ArmingCheckReporter {
public:
	ArmingCheckReporter(px4_msgs::msg::ArmingCheckReply& arming_check_reply)
		: _arming_check_reply(arming_check_reply) {}

	template<typename... Args>
	void armingCheckFailureExt(uint32_t event_id,
				events::Log log_level, const char *message, Args... args) {
		uint16_t navigation_mode_groups{};
		uint8_t health_component_index{};
		if (!addEvent(event_id, log_level, message, navigation_mode_groups,
				health_component_index, args...)) {
			printf("Error: too many events\n");
		}
	}

	void setHealth(health_component_t component, bool is_present, bool warning, bool error);

private:
	template<typename... Args>
	bool addEvent(uint32_t event_id, const events::LogLevels &log_levels, const char *message, Args... args);

	constexpr uint8_t log2(uint64_t x)
	{
		uint8_t i = 0;
		while (x > 1) {
			x >>= 1;
			++i;
		}
		return i;
	}

	px4_msgs::msg::ArmingCheckReply& _arming_check_reply;
};


template<typename... Args>
bool ArmingCheckReporter::addEvent(uint32_t event_id, const events::LogLevels &log_levels,
		const char *message, Args... args)
{
	if (_arming_check_reply.num_events >= _arming_check_reply.events.size()) {
		return false;
	}
	events::EventType& e = _arming_check_reply.events[_arming_check_reply.num_events];
	e.log_levels = ((uint8_t)log_levels.internal << 4) | (uint8_t)log_levels.external;
	e.id = event_id;
	static_assert(events::util::sizeofArguments(args...) <= sizeof(e.arguments), "Too many arguments");
	events::util::fillEventArguments((uint8_t *)e.arguments.data(), args...);
	++_arming_check_reply.num_events;
	return true;
}


inline void ArmingCheckReporter::setHealth(health_component_t component, bool is_present, bool warning, bool error)
{
	_arming_check_reply.health_component_index = log2((uint64_t)component);
	_arming_check_reply.health_component_is_present = is_present;
	_arming_check_reply.health_component_warning = warning;
	_arming_check_reply.health_component_error = error;
}
