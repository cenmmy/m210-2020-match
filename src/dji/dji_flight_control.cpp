/*! @file flight_control_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "dji_flight_control.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

/**
 * @brief monitoredTakeoff
 * 监控起飞（阻塞api调用）.这个函数确保飞行器正常起飞，只有当飞行完全起飞时返回true。
 * 推荐使用该函数保证飞行器正常起飞，除非你想在起飞时做写其他的工作，因为该函数会阻塞主线程。
 * 主要流程：
 * + 1. 订阅飞行器飞行状态和模式信息
 * + 2. 调用takeoff函数起飞
 * + 3. 检查发动机状态，两秒内每隔100ms检查一次，如果2s内发动机没有启动，则认为发动机故障
 * + 4. 检查飞行器是否在空中，11s内每隔100ms检查一次，如果超过11s还在地上，则认为繁盛故障
 * + 5. 检查飞行器模式，如果飞行模式显示起飞中，则等待1s再查询，当飞行器处在特定的模式下时认为
 * 飞行器起飞成功。
 * + 6. 取消订阅
 * @param vehicle 飞行器实例
 * @param timeout 验证订阅的超时时间，官方示例中大都设置为1
 * @return 飞行器是否正常起飞
 */
bool
monitoredTakeoff(Vehicle* vehicle, int timeout)
{
    //@todo: remove this once the getErrorCode function signature changes
    char func[50];
    int  pkgIndex;

    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, func);
        return false;
    }

    // Telemetry: Subscribe to flight status and mode at freq 10 Hz
    pkgIndex                  = 0;
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                  TOPIC_STATUS_DISPLAYMODE };
    int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
                pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
        return pkgStatus;
    }
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, func);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, timeout);
        return false;
    }

    // Start takeoff
    ACK::ErrorCode takeoffStatus = vehicle->control->takeoff(timeout);
    if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(takeoffStatus, func);
        return false;
    }

    // First check: Motors started
    int motorsNotStarted = 0;
    int timeoutCycles    = 20;

    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
           VehicleStatus::FlightStatus::ON_GROUND &&
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
           VehicleStatus::DisplayMode::MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles)
    {
        motorsNotStarted++;
        usleep(100000);
    }

    if (motorsNotStarted == timeoutCycles)
    {
        std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
        // Cleanup
        vehicle->subscribe->removePackage(0, timeout);
        return false;
    }
    else
    {
        std::cout << "Motors spinning...\n";
    }

    // Second check: In air
    int stillOnGround = 0;
    timeoutCycles     = 110;

    while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() !=
           VehicleStatus::FlightStatus::IN_AIR &&
           (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles)
    {
        stillOnGround++;
        usleep(100000);
    }

    if (stillOnGround == timeoutCycles)
    {
        std::cout << "Takeoff failed. Aircraft is still on the ground, but the "
                     "motors are spinning."
                  << std::endl;
        // Cleanup
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
            vehicle->subscribe->removePackage(0, timeout);
        }
        return false;
    }
    else
    {
        std::cout << "Ascending...\n";
    }

    // Final check: Finished takeoff
    while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
           VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
           vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
           VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF)
    {
        sleep(1);
    }

    if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_P_GPS ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
            VehicleStatus::DisplayMode::MODE_ATTITUDE)
    {
        std::cout << "Successful takeoff!\n";
    }
    else
    {
        std::cout
                << "Takeoff finished, but the aircraft is in an unexpected mode. "
                   "Please connect DJI GO.\n";
        vehicle->subscribe->removePackage(0, timeout);
        return false;
    }


    // Cleanup
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
    if (ACK::getError(ack))
    {
        std::cout
                << "Error unsubscribing; please restart the drone/FC to get back "
                   "to a clean state.\n";
    }

    return true;
}

/**
 * @brief moveByPositionOffset 指定偏移量进行移动
 * 位置控制。允许设置与当前位置的偏移量，飞行器将会根据偏移量移动到预期的位置。
 * 基本流程：
 * + 1. 设置初始点GPS和当前点GPS,一开始两者相等。
 * 通过(预期的偏移量)-(当前点与起始点的偏移量)获取当前点巨量目标点的偏移量。
 * + 2. 进入周期为20ms的循环，通过比较剩余偏移量是否大于10m，如果剩余偏移量 > 10m, 传递给
 * positionAndYawCtrl的参数为10, 如果小于10m, 则传递剩余偏移量positionAndYawCtrl。
 * 注意：在循环中我们无需循环计算剩余偏航偏移量，计算出相对与地面坐标系的欧拉角之后，将该欧拉角作为参数
 * 传递给positionAndYawCtrl即可。
 * 传递给positionAndYawCtrl的参数数值越大，其飞行器飞行的速度越快。
 * + 3. 当剩余的偏移位置小于允许的误差时，即可认为到达了预期的地点。
 * 当偏移位置小于允许误差的周期大于50个周期，即1s时，退出循环，一旦超出误差，则重新计算在范围的周期数量。
 * 剩余偏移位置大于允许误差大于10个周期，表示飞行在范围外。
 * + 4. 紧急制动1s。将速度设置为0,以防止位置质量产生任何速度。
 * + 5. 判断任务是否超时。
 *
 * @param vehicle 飞行器实例
 * @param xOffsetDesired    x轴方向上的偏移量
 * @param yOffsetDesired    y轴方向上的偏移量
 * @param zOffsetDesired    z轴方向上的偏移量
 * @param yawDesired        偏航的角度
 * @param posThresholdInM   位置误差阈值，单位为m
 * @param yawThresholdInDeg 偏航误差阈值，单位为度
 * @return 是否在规定的时间内飞到预期的位置
 * @note 通过阅读源码，认为超时时间应该根据偏移量进行设置，
 * 否则在GPS信号不好的地方，无人机会在预定位置附近旋停至超时。
 */
bool
moveByPositionOffset(Vehicle *vehicle, float xOffsetDesired,
                     float yOffsetDesired, float zOffsetDesired,
                     float yawDesired, float posThresholdInM,
                     float yawThresholdInDeg)
{
    // Set timeout: this timeout is the time you allow the drone to take to finish
    // the
    // mission
    int responseTimeout              = 1;
    // 飞行器执行任务的超时时间，单位ms
    int timeoutInMilSec              = 40000;
    // 控制频率，单位赫兹
    int controlFreqInHz              = 50;
    // 控制循环周期，1000ms(1s) / 控制频率
    int cycleTimeInMs                = 1000 / controlFreqInHz;
    // 超过控制范围的时间限制
    int outOfControlBoundsTimeLimit  = 10 * cycleTimeInMs; // 10 cycles
    int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
    int pkgIndex;

    //@todo: remove this once the getErrorCode function signature changes
    char func[50];

    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;
    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, func);
        return false;
    }

    // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
    // Hz
    pkgIndex                  = 0;
    int       freq            = 50;
    TopicName topicList50Hz[] = { TOPIC_QUATERNION, TOPIC_GPS_FUSED };
    int       numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
                pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
        return pkgStatus;
    }
    subscribeStatus =
            vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, func);
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
    }

    // Also, since we don't have a source for relative height through subscription,
    // start using broadcast height
    if (!startGlobalPositionBroadcast(vehicle))
    {
        // Cleanup before return
        vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        return false;
    }

    // Wait for data to come in
    sleep(1);

    // Get data

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition currentBroadcastGP;
    Telemetry::GlobalPosition originBroadcastGP;

    // Convert position offset from first position to local coordinates
    Telemetry::Vector3f localOffset;

    currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    originSubscriptionGPS  = currentSubscriptionGPS;
    localOffsetFromGpsOffset(vehicle, localOffset,
                             static_cast<void*>(&currentSubscriptionGPS),
                             static_cast<void*>(&originSubscriptionGPS));

    // Get the broadcast GP since we need the height for zCmd
    currentBroadcastGP = vehicle->broadcast->getGlobalPosition();

    // Get initial offset. We will update this in a loop later.
    double xOffsetRemaining = xOffsetDesired - localOffset.x;
    double yOffsetRemaining = yOffsetDesired - localOffset.y;
    double zOffsetRemaining = zOffsetDesired - localOffset.z;

    // Conversions
    double yawDesiredRad     = DEG2RAD * yawDesired;
    double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

    //! Get Euler angle

    // Quaternion retrieved via subscription
    Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
    // Quaternion retrieved via broadcast
    Telemetry::Quaternion broadcastQ;

    double yawInRad;
    subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
    yawInRad = toEulerAngle((static_cast<void*>(&subscriptionQ))).z / DEG2RAD;

    int   elapsedTimeInMs     = 0;
    int   withinBoundsCounter = 0;
    int   outOfBounds         = 0;
    int   brakeCounter        = 0;
    int   speedFactor         = 10;
    float xCmd, yCmd, zCmd;

    /*! Calculate the inputs to send the position controller. We implement basic
   *  receding setpoint position control and the setpoint is always 1 m away
   *  from the current position - until we get within a threshold of the goal.
   *  From that point on, we send the remaining distance as the setpoint.
   */
    if (xOffsetDesired > 0)
        xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
    else if (xOffsetDesired < 0)
        xCmd =
                (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
    else
        xCmd = 0;

    if (yOffsetDesired > 0)
        yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
    else if (yOffsetDesired < 0)
        yCmd =
                (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
    else
        yCmd = 0;

    zCmd = currentBroadcastGP.height + zOffsetDesired;

    //! Main closed-loop receding setpoint position control
    while (elapsedTimeInMs < timeoutInMilSec)
    {
        vehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd,
                                             yawDesiredRad / DEG2RAD);

        usleep(cycleTimeInMs * 1000);
        elapsedTimeInMs += cycleTimeInMs;

        //! Get current position in required coordinates and units
        subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
        yawInRad      = toEulerAngle((static_cast<void*>(&subscriptionQ))).z;
        currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        localOffsetFromGpsOffset(vehicle, localOffset,
                                 static_cast<void*>(&currentSubscriptionGPS),
                                 static_cast<void*>(&originSubscriptionGPS));

        // Get the broadcast GP since we need the height for zCmd
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();

        //! See how much farther we have to go
        xOffsetRemaining = xOffsetDesired - localOffset.x;
        yOffsetRemaining = yOffsetDesired - localOffset.y;
        zOffsetRemaining = zOffsetDesired - localOffset.z;

        //! See if we need to modify the setpoint
        if (std::abs(xOffsetRemaining) < speedFactor)
        {
            xCmd = xOffsetRemaining;
        }
        if (std::abs(yOffsetRemaining) < speedFactor)
        {
            yCmd = yOffsetRemaining;
        }

        if (std::abs(xOffsetRemaining) < posThresholdInM &&
                 std::abs(yOffsetRemaining) < posThresholdInM &&
                 std::abs(zOffsetRemaining) < posThresholdInM &&
                 std::abs(std::abs(yawInRad) - std::abs(yawDesiredRad)) < yawThresholdInRad)
        {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
        }
        else
        {
            if (withinBoundsCounter != 0)
            {
                //! 2. Start incrementing an out-of-bounds counter
                outOfBounds += cycleTimeInMs;
            }
        }
        //! 3. Reset withinBoundsCounter if necessary
        if (outOfBounds > outOfControlBoundsTimeLimit)
        {
            withinBoundsCounter = 0;
            outOfBounds         = 0;
        }
        //! 4. If within bounds, set flag and break
        if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
        {
            break;
        }
    }

    //! Set velocity to zero, to prevent any residual velocity from position
    //! command
    // 紧急制动1s。将速度设置为0,以防止位置质量产生任何速度。
    while (brakeCounter < withinControlBoundsTimeReqmt)
    {
        vehicle->control->emergencyBrake();
        usleep(cycleTimeInMs * 10);
        brakeCounter += cycleTimeInMs;
    }
    // 判断任务是否超时
    if (elapsedTimeInMs >= timeoutInMilSec)
    {
        std::cout << "Task timeout!\n";
        ACK::ErrorCode ack =
                vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        if (ACK::getError(ack))
        {
            std::cout << "Error unsubscribing; please restart the drone/FC to get "
                         "back to a clean state.\n";
        }
        return ACK::FAIL;
    }

    ACK::ErrorCode ack =
            vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack))
    {
        std::cout
                << "Error unsubscribing; please restart the drone/FC to get back "
                   "to a clean state.\n";
    }

    return ACK::SUCCESS;
}

/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool
monitoredLanding(Vehicle* vehicle, int timeout)
{
    //@todo: remove this once the getErrorCode function signature changes
    char func[50];
    int  pkgIndex;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        // Telemetry: Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            return false;
        }

        // Telemetry: Subscribe to flight status and mode at freq 10 Hz
        pkgIndex                  = 0;
        int       freq            = 10;
        TopicName topicList10Hz[] = { TOPIC_STATUS_FLIGHT,
                                      TOPIC_STATUS_DISPLAYMODE };
        int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
        bool enableTimestamp = false;

        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
                    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
        if (!(pkgStatus))
        {
            return pkgStatus;
        }
        subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            // Cleanup before return
            vehicle->subscribe->removePackage(pkgIndex, timeout);
            return false;
        }
    }

    // Start landing
    ACK::ErrorCode landingStatus = vehicle->control->land(timeout);
    if (ACK::getError(landingStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(landingStatus, func);
        return false;
    }

    // First check: Landing started
    int landingNotStarted = 0;
    int timeoutCycles     = 20;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
               VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
               landingNotStarted < timeoutCycles)
        {
            landingNotStarted++;
            usleep(100000);
        }
    }
    else if (vehicle->isM100())
    {
        while (vehicle->broadcast->getStatus().flight !=
               DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING &&
               landingNotStarted < timeoutCycles)
        {
            landingNotStarted++;
            usleep(100000);
        }
    }

    if (landingNotStarted == timeoutCycles)
    {
        std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
            // Cleanup before return
            ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
            if (ACK::getError(ack)) {
                std::cout << "Error unsubscribing; please restart the drone/FC to get "
                             "back to a clean state.\n";
            }
        }
        return false;
    }
    else
    {
        std::cout << "Landing...\n";
    }

    // Second check: Finished landing
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() ==
               VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
               vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() ==
               VehicleStatus::FlightStatus::IN_AIR)
        {
            sleep(1);
        }

        if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_P_GPS ||
                vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() !=
                VehicleStatus::DisplayMode::MODE_ATTITUDE)
        {
            std::cout << "Successful landing!\n";
        }
        else
        {
            std::cout
                    << "Landing finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI GO.\n";
            ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
            if (ACK::getError(ack))
            {
                std::cout << "Error unsubscribing; please restart the drone/FC to get "
                             "back to a clean state.\n";
            }
            return false;
        }
    }
    else if (vehicle->isLegacyM600())
    {
        while (vehicle->broadcast->getStatus().flight >
               DJI::OSDK::VehicleStatus::FlightStatus::STOPED)
        {
            sleep(1);
        }

        Telemetry::GlobalPosition gp;
        do
        {
            sleep(2);
            gp = vehicle->broadcast->getGlobalPosition();
        } while (gp.altitude != 0);

        if (gp.altitude != 0)
        {
            std::cout
                    << "Landing finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI GO.\n";
            return false;
        }
        else
        {
            std::cout << "Successful landing!\n";
        }
    }
    else // M100
    {
        while (vehicle->broadcast->getStatus().flight ==
               DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING)
        {
            sleep(1);
        }

        Telemetry::GlobalPosition gp;
        do
        {
            sleep(2);
            gp = vehicle->broadcast->getGlobalPosition();
        } while (gp.altitude != 0);

        if (gp.altitude != 0)
        {
            std::cout
                    << "Landing finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI GO.\n";
            return false;
        }
        else
        {
            std::cout << "Successful landing!\n";
        }
    }

    // Cleanup
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack))
        {
            std::cout
                    << "Error unsubscribing; please restart the drone/FC to get back "
                       "to a clean state.\n";
        }
    }

    return true;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates.
    Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(Vehicle* vehicle, Telemetry::Vector3f& deltaNed,
                         void* target, void* origin)
{
    Telemetry::GPSFused*       subscriptionTarget;
    Telemetry::GPSFused*       subscriptionOrigin;
    Telemetry::GlobalPosition* broadcastTarget;
    Telemetry::GlobalPosition* broadcastOrigin;
    double                     deltaLon;
    double                     deltaLat;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        subscriptionTarget = (Telemetry::GPSFused*)target;
        subscriptionOrigin = (Telemetry::GPSFused*)origin;
        deltaLon   = subscriptionTarget->longitude - subscriptionOrigin->longitude;
        deltaLat   = subscriptionTarget->latitude - subscriptionOrigin->latitude;
        deltaNed.x = deltaLat * C_EARTH;
        deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
        deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
    }
    else
    {
        broadcastTarget = (Telemetry::GlobalPosition*)target;
        broadcastOrigin = (Telemetry::GlobalPosition*)origin;
        deltaLon        = broadcastTarget->longitude - broadcastOrigin->longitude;
        deltaLat        = broadcastTarget->latitude - broadcastOrigin->latitude;
        deltaNed.x      = deltaLat * C_EARTH;
        deltaNed.y      = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
        deltaNed.z      = broadcastTarget->altitude - broadcastOrigin->altitude;
    }
}

Telemetry::Vector3f
toEulerAngle(void* quaternionData)
{
    Telemetry::Vector3f    ans;
    Telemetry::Quaternion* quaternion = (Telemetry::Quaternion*)quaternionData;

    double q2sqr = quaternion->q2 * quaternion->q2;
    double t0    = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
    double t1 =
            +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
    double t2 =
            -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
    double t3 =
            +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
    double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;

    ans.x = asin(t2);
    ans.y = atan2(t3, t4);
    ans.z = atan2(t1, t0);

    return ans;
}

bool startGlobalPositionBroadcast(Vehicle* vehicle)
{
    uint8_t freq[16];

    /* Channels definition for A3/N3/M600
   * 0 - Timestamp
   * 1 - Attitude Quaternions
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - GPS Detailed Information
   * 7 - RTK Detailed Information
   * 8 - Magnetometer
   * 9 - RC Channels Data
   * 10 - Gimbal Data
   * 11 - Flight Status
   * 12 - Battery Level
   * 13 - Control Information
   */
    freq[0]  = DataBroadcast::FREQ_HOLD;
    freq[1]  = DataBroadcast::FREQ_HOLD;
    freq[2]  = DataBroadcast::FREQ_HOLD;
    freq[3]  = DataBroadcast::FREQ_HOLD;
    freq[4]  = DataBroadcast::FREQ_HOLD;
    freq[5]  = DataBroadcast::FREQ_50HZ; // This is the only one we want to change
    freq[6]  = DataBroadcast::FREQ_HOLD;
    freq[7]  = DataBroadcast::FREQ_HOLD;
    freq[8]  = DataBroadcast::FREQ_HOLD;
    freq[9]  = DataBroadcast::FREQ_HOLD;
    freq[10] = DataBroadcast::FREQ_HOLD;
    freq[11] = DataBroadcast::FREQ_HOLD;
    freq[12] = DataBroadcast::FREQ_HOLD;
    freq[13] = DataBroadcast::FREQ_HOLD;

    ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreq(freq, 1);
    if (ACK::getError(ack))
    {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    else
    {
        return true;
    }
}
