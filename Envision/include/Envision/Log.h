#pragma once
#include "PCH/pch.h"

namespace Envision {

	class Log
	{
	public:
		static void Init();

		inline static std::shared_ptr<spdlog::logger>& GetCoreLogger() { return s_CoreLogger; }
		inline static std::shared_ptr<spdlog::logger>& GetClientLogger() { return s_ClientLogger; }
	private:
		static std::shared_ptr<spdlog::logger> s_CoreLogger;
		static std::shared_ptr<spdlog::logger> s_ClientLogger;
	};
}

// Core Log macros
#define EN_CORE_TRACE(...)	::Envision::Log::GetCoreLogger()->trace(__VA_ARGS__)
#define EN_CORE_DEBUG(...)  ::Envision::Log::GetCoreLogger()->debug(__VA_ARGS__)
#define EN_CORE_INFO(...)		::Envision::Log::GetCoreLogger()->info(__VA_ARGS__)
#define EN_CORE_WARN(...)		::Envision::Log::GetCoreLogger()->warn(__VA_ARGS__)
#define EN_CORE_ERROR(...)	::Envision::Log::GetCoreLogger()->error(__VA_ARGS__)

// Client Log macros
#define EN_CLIENT_TRACE(...)::Envision::Log::GetClientLogger()->trace(__VA_ARGS__)
#define EN_CLIENT_DEBUG(...)::Envision::Log::GetClientLogger()->debug(__VA_ARGS__)
#define EN_CLIENT_INFO(...)	::Envision::Log::GetClientLogger()->info(__VA_ARGS__)
#define EN_CLIENT_WARN(...)	::Envision::Log::GetClientLogger()->warn(__VA_ARGS__)
#define EN_CLIENT_ERROR(...)::Envision::Log::GetClientLogger()->error(__VA_ARGS__)
