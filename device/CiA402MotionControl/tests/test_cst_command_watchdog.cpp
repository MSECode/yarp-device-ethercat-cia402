// SPDX-License-Identifier: BSD-3-Clause
#include "../CstCommandWatchdog.h"
#include <iostream>
#include <limits>
#include <stdexcept>

using Guard = yarp::dev::detail::CstCommandWatchdog;
using Mode = Guard::Mode;
namespace {
int checks = 0;
void require(bool result, const char* message)
{
    ++checks;
    if (!result) throw std::runtime_error(message);
}
Guard::TimePoint at(int milliseconds)
{
    return Guard::TimePoint{} + std::chrono::milliseconds(milliseconds);
}
bool command(Guard& guard, Mode mode, int milliseconds)
{
    return guard.mayAccept(mode, at(milliseconds)) &&
           guard.recordAcceptedCommand(at(milliseconds));
}
}
int main()
{
    try {
        for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
                               std::numeric_limits<double>::quiet_NaN()}) {
            bool rejected = false;
            try { Guard guard(invalid); } catch (const std::invalid_argument&) { rejected = true; }
            require(rejected, "invalid timeout accepted");
        }
        {
            Guard guard;
            require(!guard.enabled(), "legacy default is not disabled");
            require(command(guard, Mode::Torque, 0), "disabled guard rejected command");
            require(!guard.poll(Mode::Torque, at(100000)), "disabled timeout expired");
        }
        for (Mode mode : {Mode::Torque, Mode::Current}) {
            Guard guard(.1);
            require(!guard.poll(mode, at(0)), "mode entry did not allow startup grace");
            require(!guard.poll(mode, at(99)), "expired before startup deadline");
            require(guard.poll(mode, at(100)), "missing FIRST command did not expire");
            require(!command(guard, mode, 101), "late command cleared expiry");
            guard.disarm();
            require(guard.expired(), "inactive mode cleared the fault latch");
            require(!command(guard, mode, 102), "mode re-entry bypassed explicit acknowledgement");
            guard.acknowledgeIdle();
            require(!guard.expired(), "explicit IDLE did not clear latch");
            require(command(guard, mode, 103), "fresh command after explicit re-arm rejected");
            require(!guard.poll(mode, at(202)), "fresh command expired early");
            require(guard.poll(mode, at(203)), "stopped producer did not expire");
        }
        {
            Guard guard(.1);
            // The SAME numerical target remains valid if actually received again.
            for (int t = 0; t <= 1000; t += 10)
                require(command(guard, Mode::Torque, t), "fresh streaming commands rejected");
            // Driver polling/PDO retransmission never renews the producer lease.
            for (int t = 1001; t < 1100; ++t)
                require(!guard.poll(Mode::Torque, at(t)), "expired early during repeated PDOs");
            require(guard.poll(Mode::Torque, at(1100)), "repeated PDOs kept frozen producer alive");
            require(!command(guard, Mode::Torque, 1101), "late packet automatically re-armed");
        }
        {
            Guard guard(.1);
            require(command(guard, Mode::Torque, 0), "initial torque rejected");
            require(!guard.poll(Mode::Current, at(90)), "CST flavor change failed");
            require(guard.poll(Mode::Current, at(190)), "old torque refreshed CURRENT deadline");
        }
        {
            Guard guard(.1);
            require(command(guard, Mode::Torque, 0), "initial command rejected");
            guard.trip(); // PDO transport failure: same sticky inhibit as expiry.
            require(!command(guard, Mode::Torque, 1), "transport fault accepted new target");
            require(guard.poll(Mode::Inactive, at(2)), "transport fault auto-cleared in IDLE");
            guard.acknowledgeIdle();
            require(command(guard, Mode::Current, 3), "explicit recovery failed");
        }
        {
            Guard guard(.1);
            require(command(guard, Mode::Torque, 50), "initial command rejected");
            require(guard.poll(Mode::Torque, at(49)), "backward host time accepted");
        }
        {
            Guard first(.1), second(.1);
            require(command(first, Mode::Torque, 0), "first axis rejected");
            require(command(second, Mode::Torque, 0), "second axis rejected");
            second.trip();
            const bool batchValid = first.mayAccept(Mode::Torque, at(90)) &&
                                    second.mayAccept(Mode::Torque, at(90));
            require(!batchValid, "faulted group command accepted");
            // No recordAcceptedCommand for either axis on a failed batch.
            require(first.poll(Mode::Torque, at(100)), "failed batch refreshed old first-axis target");
        }
        std::cout << "PASS: " << checks << " CST command-watchdog checks (fake clock, no hardware)\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
