#include "tai_clock.hpp"
#include <stdexcept>
#include <time.h>
#include <sys/timex.h> /* < only available on Linux */

namespace {

// POSIX seconds representing 2004-01-01 (UTC)
static const auto its_epoch_posix = std::chrono::seconds(1072915200);

} // namespace

tai_clock::time_point tai_clock::now()
{
    using scale_second = std::ratio_divide<std::ratio<1>, period>;
    using scale_nanosecond = std::ratio_divide<std::nano, period>;

    timespec now;
    if (clock_gettime(CLOCK_TAI, &now) == 0) {
        // tai_clock uses microsecond ticks
        tai_clock::rep ticks = 0;
        ticks = now.tv_sec * scale_second::num / scale_second::den;
        ticks += now.tv_nsec * scale_nanosecond::num / scale_nanosecond::den;
        return tai_clock::time_point(duration(ticks));
    } else {
        throw std::runtime_error("reading CLOCK_TAI failed");
    }
}

bool tai_clock::synchronized()
{
    timex ntx;
    ntx.modes = 0;
    if (adjtimex(&ntx) == TIME_ERROR) {
        return false;
    } else {
        return ntx.tai != 0;
    }
}

template<>
vanetza::Clock::time_point time_point_cast<vanetza::Clock>(tai_clock::time_point tai)
{
    // TAI duration since POSIX epoch (1970-01-01 UTC)
    auto tai_since_epoch = tai.time_since_epoch();
    return vanetza::Clock::time_point { tai_since_epoch - its_epoch_posix };
}

template<>
tai_clock::time_point time_point_cast<tai_clock>(vanetza::Clock::time_point its)
{
    // TAI duration since ITS epoch (2004-01-01 UTC)
    auto its_since_epoch = its.time_since_epoch();
    return tai_clock::time_point { its_since_epoch + its_epoch_posix };
}
