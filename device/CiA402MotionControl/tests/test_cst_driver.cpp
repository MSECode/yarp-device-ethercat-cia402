// SPDX-License-Identifier: BSD-3-Clause
// Include the implementation only in this separate test executable, NOT the
// plugin. This exposes Impl to its friend without exposing it to API users.
#include "../CiA402MotionControl.cpp"
#include <yarp/os/Network.h>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace yarp::dev {
struct CiA402MotionControlTestAccess
{
    static void setup(CiA402MotionControl& driver, int mode)
    {
        auto& impl = *driver.m_impl;
        impl.numAxes = 2;
        impl.setPoints.resize(2);
        impl.setPoints.watchdogs.assign(2, detail::CstCommandWatchdog(.1));
        impl.controlModeState.resize(2);
        impl.controlModeState.active.assign(2, mode);
        impl.controlModeState.target.assign(2, mode);
        impl.trqLatched.assign(2, false);
        impl.currLatched.assign(2, false);
    }
    static void age(CiA402MotionControl& driver, int axis, int milliseconds)
    {
        auto& impl = *driver.m_impl;
        auto& guard = impl.setPoints.watchdogs[axis];
        guard.acknowledgeIdle();
        const auto old = detail::CstCommandWatchdog::Clock::now() -
                         std::chrono::milliseconds(milliseconds);
        guard.poll(CiA402MotionControl::Impl::watchdogMode(impl.controlModeState.active[axis]), old);
        guard.recordAcceptedCommand(old);
    }
    static void poll(CiA402MotionControl& driver) { driver.m_impl->checkCstWatchdogs(); }
    static void transportFailure(CiA402MotionControl& driver) { driver.m_impl->stopCstOnTransportFailure(); }
    static int target(const CiA402MotionControl& driver, int axis)
    { return driver.m_impl->controlModeState.target[axis]; }
    static bool latched(const CiA402MotionControl& driver, int axis)
    { return driver.m_impl->setPoints.watchdogs[axis].expired(); }
    static double value(const CiA402MotionControl& driver, int axis, int mode)
    {
        return mode == VOCAB_CM_TORQUE ? driver.m_impl->setPoints.jointTorques[axis] :
                                       driver.m_impl->setPoints.motorCurrents[axis];
    }
    static bool hasTarget(const CiA402MotionControl& driver, int axis, int mode)
    {
        return mode == VOCAB_CM_TORQUE ? driver.m_impl->setPoints.hasTorqueSP[axis] :
                                       driver.m_impl->setPoints.hasCurrentSP[axis];
    }
    static void actualIdle(CiA402MotionControl& driver, int axis)
    { driver.m_impl->controlModeState.active[axis] = VOCAB_CM_IDLE; }
    static void actualMode(CiA402MotionControl& driver, int axis, int mode)
    { driver.m_impl->controlModeState.active[axis] = mode; }
    static bool expiresAt(CiA402MotionControl& driver, int axis, int futureMilliseconds)
    {
        auto& impl = *driver.m_impl;
        return impl.setPoints.watchdogs[axis].poll(
            CiA402MotionControl::Impl::watchdogMode(impl.controlModeState.active[axis]),
            detail::CstCommandWatchdog::Clock::now() + std::chrono::milliseconds(futureMilliseconds));
    }
};
}
using Driver = yarp::dev::CiA402MotionControl;
using Access = yarp::dev::CiA402MotionControlTestAccess;
namespace {
int checks = 0;
void require(bool result, const char* message)
{
    ++checks;
    if (!result) throw std::runtime_error(message);
}
bool scalar(Driver& d, int mode, int axis, double value)
{
    return mode == VOCAB_CM_TORQUE ? d.setRefTorque(axis, value) :
                                    d.setRefCurrent(axis, value);
}
bool all(Driver& d, int mode, const double* values)
{
    return mode == VOCAB_CM_TORQUE ? d.setRefTorques(values) :
                                    d.setRefCurrents(values);
}
bool group(Driver& d, int mode, const int* axes, const double* values)
{
    return mode == VOCAB_CM_TORQUE ? d.setRefTorques(2, axes, values) :
                                    d.setRefCurrents(2, axes, values);
}
}
int main()
{
    yarp::os::Network network;
    yarp::os::Network::setLocalMode(true);
    try {
        for (int mode : {VOCAB_CM_TORQUE, VOCAB_CM_CURRENT}) {
            Driver d;
            Access::setup(d, mode); // No open(), periodic thread, NIC, or drive.
            require(scalar(d, mode, 0, .2), "scalar command failed");
            require(Access::value(d, 0, mode) == .2, "scalar target not stored");
            const double values[] = {.1, -.2};
            require(all(d, mode, values), "all-axis setter failed");
            const int axes[] = {1, 0};
            require(group(d, mode, axes, values), "group setter failed");
            require(Access::value(d, 0, mode) == -.2, "group ordering changed");
            require(Access::value(d, 1, mode) == .1, "group ordering changed");
            require(!scalar(d, mode, -1, .1), "negative axis accepted");
            require(!scalar(d, mode, 2, .1), "out-of-range axis accepted");
            require(!scalar(d, mode, 0, std::numeric_limits<double>::quiet_NaN()), "NaN accepted");
            require(!all(d, mode, nullptr), "null targets accepted");
            const double invalid[] = {.3, std::numeric_limits<double>::infinity()};
            require(!all(d, mode, invalid), "infinite batch accepted");
            require(Access::value(d, 0, mode) == -.2, "invalid batch partially changed target");

            // Stop/freeze the producer while the driver's watchdog polling keeps running.
            Access::age(d, 0, 200);
            Access::poll(d);
            require(Access::latched(d, 0), "expired reference did not latch");
            require(Access::target(d, 0) == VOCAB_CM_IDLE, "expiry did not request IDLE");
            require(Access::value(d, 0, mode) == 0., "expiry did not clear cached target");
            require(!Access::hasTarget(d, 0, mode), "expiry retained stale-target flag");
            require(!scalar(d, mode, 0, .1), "late scalar command bypassed expiry");
            require(!all(d, mode, values), "all-axis command bypassed expiry");
            require(!group(d, mode, axes, values), "group command bypassed expiry");
            require(!d.setControlMode(0, mode), "CST request automatically re-armed");
            int modes[] = {mode, mode};
            require(!d.setControlModes(modes), "all-axis modes bypassed expiry");
            require(!d.setControlModes(2, axes, modes), "group modes bypassed expiry");
            require(Access::latched(d, 0), "rejected request cleared fault");
            require(d.setControlMode(0, VOCAB_CM_IDLE), "explicit IDLE rejected");
            Access::actualIdle(d, 0); // Simulate statusword acknowledgement.
            require(d.setControlMode(0, mode), "deliberate re-arm rejected");
            require(!scalar(d, mode, 0, .1), "command accepted before actual mode acknowledgement");
            Access::actualMode(d, 0, mode);
            require(scalar(d, mode, 0, .1), "fresh command after re-arm rejected");

            // A rejected group must not renew another axis's old reference.
            Access::setup(d, mode);
            require(all(d, mode, values), "batch test setup failed");
            Access::age(d, 0, 50);
            Access::age(d, 1, 200);
            require(!all(d, mode, values), "expired-axis batch accepted");
            require(Access::expiresAt(d, 0, 60), "rejected batch refreshed first axis");
        }
        {
            Driver d;
            Access::setup(d, VOCAB_CM_TORQUE);
            require(d.setRefTorque(0, .2), "transport setup failed");
            Access::transportFailure(d);
            require(Access::latched(d, 0), "transport failure did not latch");
            require(Access::target(d, 0) == VOCAB_CM_IDLE, "transport failure did not request IDLE");
            require(Access::value(d, 0, VOCAB_CM_TORQUE) == 0., "transport failure left old target");
            require(!d.setRefTorque(0, .2), "command after transport failure accepted");
            CiA402::StateMachine sm;
            const auto command = sm.update(0x0027, 10, 0, false);
            require(command.controlword == 0 && command.opMode == 0 && command.writeOpMode,
                    "IDLE did not produce voltage-disable controlword");
        }
        std::cout << "PASS: " << checks << " actual driver CST integration checks (no EtherCAT opened)\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
