#ifndef TAI_CLOCK_HPP
#define TAI_CLOCK_HPP

#include <vanetza/common/clock.hpp>
#include <chrono>
#include <cstdint>

// C++20 will provide a std::chrono::tai_clock. Awesome, any flux compensator in 2018?

struct tai_clock
{
    using rep = int64_t;
    using period = std::micro; /*< microsecond ticks */
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<tai_clock>;

    /**
     * CLOCK_TAI is affected by discontinuous jumps of system time (UTC).
     * Kernel's tai offset may get adjusted during runtime.
     * -> CLOCK_TAI is not steady
     */
    static constexpr bool is_steady() { return false; }

    /**
     * Return current TAI time point.
     * This may throw an exception if CLOCK_TAI is unavailable.
     * \return TAI time point
     */
    static time_point now();

    /**
     * Check if TAI clock is sychronized, i.e. local clock is stabilized and TAI offset is set
     * \return true if sychronized
     */
    static bool synchronized();
};

template<class ToClock, class FromTimePoint>
typename ToClock::time_point time_point_cast(FromTimePoint);

template<>
vanetza::Clock::time_point time_point_cast<vanetza::Clock>(tai_clock::time_point);

template<>
tai_clock::time_point time_point_cast<tai_clock>(vanetza::Clock::time_point);

#endif /* TAI_CLOCK_HPP */
