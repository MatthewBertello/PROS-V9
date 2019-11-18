#pragma once
#ifndef positionTracker_h
#define positionTracker_h

#define _USE_MATH_DEFINES

#include <cmath>
#include "pros/rtos.hpp"
#include "headers/robotDrive.h"

class positionTracker
{

    public:

    robotDrive *driveTracking;
    float currentAngle = 0;
    float currentX = 0;
    float currentY = 0;
    float previousLeft = 0;
    float previousRight = 0;
    float previousStrafe = 0;

    float angleVelocity = 0;
    float xVelocity = 0;
    float yVelocity = 0;
    float previousAngle = 0;
    float previousX = 0;
    float previousY = 0;

    float velocityRefresh;

    uint32_t previousVelocityCheck = pros::c::millis();

    float getwheelDistanceMoved(float rotationsMoved, float wheelDiameter, float ticksPerRotation)
    {
        return wheelDiameter * M_PI / ticksPerRotation;
    }

    void trackVelocity()
    {
        if (pros::c::millis() - previousVelocityCheck > velocityRefresh)
        {
            angleVelocity = ((currentAngle - previousAngle) * 1000) / previousVelocityCheck;
            xVelocity = ((currentX - previousX) * 1000) / previousVelocityCheck;
            yVelocity = ((currentY - previousY) * 1000) / previousVelocityCheck;

            previousAngle = currentAngle;
            previousX = currentX;
            previousY = currentY;
        }
    }

    void trackPosition()
    {
        float left = driveTracking->getAbsoluteLeftDriveSensor();
        float right = driveTracking->getAbsoluteRightDriveSensor();
        float strafe = driveTracking->getAbsoluteStrafeDriveSensor();
        float leftWheelDistance = getwheelDistanceMoved(left - previousLeft, driveTracking->leftWheelDiameter, driveTracking->leftTicksPerRotation);
        float rightWheelDistance = getwheelDistanceMoved(right - previousRight, driveTracking->rightWheelDiameter, driveTracking->rightTicksPerRotation);
        float strafeWheelDistance = getwheelDistanceMoved(strafe - previousStrafe, driveTracking->strafeWheelDiameter, driveTracking->strafeTicksPerRotation);

        previousLeft = left;
        previousRight = right;
        previousStrafe = strafe;

        float angleChange = (leftWheelDistance - rightWheelDistance) / (driveTracking->leftDistanceFromCenter + driveTracking->rightDistanceFromCenter);
        float distanceTraveled;
        float distanceTraveledStrafe;
        float halfAngleChange = angleChange / 2;

        if (angleChange == 0)
        {
            distanceTraveled = rightWheelDistance;
            distanceTraveledStrafe = strafeWheelDistance;
        }
        else
        {
            float rightRadius = rightWheelDistance / angleChange;
            float sinHalfAngleChange = sin(halfAngleChange);
            distanceTraveled = ((rightRadius + driveTracking->rightDistanceFromCenter) * sinHalfAngleChange) * 2.0;

            float strafeRadius = strafeWheelDistance / angleChange;
            distanceTraveledStrafe = ((strafeRadius + driveTracking->strafeDistanceFromCenter) * sinHalfAngleChange) * 2.0;
        }

        float cosAngleTraveled = cos(halfAngleChange + currentAngle);
        float sinAngleTraveled = sin(halfAngleChange + currentAngle);

        currentY += distanceTraveled * cosAngleTraveled;
        currentX += distanceTraveled * sinAngleTraveled;

        currentY += distanceTraveledStrafe * cosAngleTraveled;
        currentX += distanceTraveledStrafe * sinAngleTraveled;

        currentAngle += angleChange;
    }
};
#endif