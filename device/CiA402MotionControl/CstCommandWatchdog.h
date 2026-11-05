// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace yarp::dev::detail
{
// Pure timing state, protected by the driver's setpoint mutex. No hardware or
// sleeping is needed to test a stopped producer while PDO exchange continues.
class CstCommandWatchdog
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    enum class Mode { Inactive, Torque, Current };

    explicit CstCommandWatchdog(double timeoutSec = 0.0) : m_timeoutSec(timeoutSec)
    {
        if (!std::isfinite(timeoutSec) || timeoutSec < 0.0)
            throw std::invalid_argument("CST command timeout must be finite and non-negative");
    }

    bool expired() const noexcept { return m_expired; }
    bool enabled() const noexcept { return m_timeoutSec > 0.0; }

    // Losing/changing a mode discards its command history, NOT a fault latch.
    void disarm() noexcept
    {
        m_mode = Mode::Inactive;
        m_entered.reset();
        m_lastCommand.reset();
    }

    void acknowledgeIdle() noexcept
    {
        disarm();
        m_expired = false;
    }

    void trip() noexcept { m_expired = true; }

    bool poll(Mode mode, TimePoint now) noexcept
    {
        if (m_expired) return true;
        if (mode == Mode::Inactive) {
            disarm();
            return false;
        }
        if (mode != m_mode) {
            m_mode = mode;
            m_entered = now;
            m_lastCommand.reset();
        }
        const TimePoint baseline = m_lastCommand.value_or(*m_entered);
        if (now < baseline ||
            (enabled() && std::chrono::duration<double>(now - baseline).count() >= m_timeoutSec))
            trip();
        return m_expired;
    }

    // Validate every axis first when accepting a group command. A failed batch
    // must not refresh another axis's OLD target.
    bool mayAccept(Mode mode, TimePoint now) noexcept
    {
        return mode != Mode::Inactive && !poll(mode, now);
    }

    bool recordAcceptedCommand(TimePoint now) noexcept
    {
        if (m_expired || m_mode == Mode::Inactive || !m_entered) return false;
        const TimePoint baseline = m_lastCommand.value_or(*m_entered);
        if (now < baseline) {
            trip();
            return false;
        }
        m_lastCommand = now;
        return true;
    }

private:
    double m_timeoutSec;
    Mode m_mode{Mode::Inactive};
    bool m_expired{false};
    std::optional<TimePoint> m_entered;
    std::optional<TimePoint> m_lastCommand;
};
}
