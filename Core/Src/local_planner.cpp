/*
 * local_planner.cpp
 *
 *  Created on: Jul 17, 2024
 *      Author: tu
 */

#include "local_planner.h"

float deltaTime = 0.001;
/* bot param */
float botPositionX = 0;
float botPositionY = 0;
float goalDistance;

/* velocity param*/
float VelocityNow;
float xMoved = 0.0, yMoved = 0.0, Moved = 0.0;
float remain = goalDistance;
float x_vec, y_vec;

float maxVelocity = 0.4;
float vel_0 = 0.05;
float vel_1 = maxVelocity - 0.05;
float vel_2 = maxVelocity;
float dist_0 = 0.05;
float dist_1 = maxVelocity / 1.5 + 0.1 / 3 - 0.05;
float dist_2 = maxVelocity / 1.5 + 0.1 / 3;

bool hasObs = false;
float theta, d1, d2, x, y;
pair p1(x, y), p2(x, y);

void cmd_vel_pub(float Vx_, float Vy_, float W_)
{
    Vx = (double)Vx_;
    Vy = (double)Vy_;
    W = (double)W_;
}
void updateUnitVector(double moved)
{
    if (moved < d1)
    {
    }
    else if (moved >= d1 && goalDistance - moved > d2)
    {
    }
    else
    {
    }
    // x_vec = (xGoal - botPositionX) / goalDistance;
    // y_vec = (yGoal - botPositionY) / goalDistance;
}
void planNewPath(std::vector<int> obsOnRoad, const float xGoal, const float yGoal)
{
    float D1, D2, radius;
    radius = R + obsticals[obsOnRoad[0]].w;
    D1 = hypot((obsticals[obsOnRoad[0]].x - botPositionX), (obsticals[obsOnRoad[0]].y - botPositionY));
    d1 = sqrt(pow(D1, 2) - pow(radius, 2));
    D2 = hypot((obsticals[obsOnRoad[0]].x - xGoal), (obsticals[obsOnRoad[0]].y - yGoal));
    d2 = sqrt(pow(D2, 2) - pow(radius, 2));

    x_vec = ((obsticals[obsOnRoad[0]].x - botPositionX) * d1 / D1 + (obsticals[obsOnRoad[0]].y - botPositionY) * -radius / D1) / D1;
    y_vec = ((obsticals[obsOnRoad[0]].x - botPositionX) * radius / D1 + (obsticals[obsOnRoad[0]].y - botPositionY) * d1 / D1) / D1;
    p1.first = x_vec * d1 + botPositionX;
    p1.second = y_vec * d1 + botPositionY;
    p2.first = ((obsticals[obsOnRoad[0]].x - xGoal) * d2 / D2 + (obsticals[obsOnRoad[0]].y - yGoal) * radius / D2) / D2 * d2 + xGoal;
    p2.second = ((obsticals[obsOnRoad[0]].x - xGoal) * -radius / D1 + (obsticals[obsOnRoad[0]].y - yGoal) * d2 / D2) / D2 * d2 + yGoal;
    theta = acos(((p1.first - obsticals[obsOnRoad[0]].x) * (p2.first - obsticals[obsOnRoad[0]].x) + (p1.second - obsticals[obsOnRoad[0]].y) * (p2.second - obsticals[obsOnRoad[0]].y)) / pow(radius, 2));
    goalDistance = d1 + d2 + radius * theta;
}

void pointToDist(const float xGoal, const float yGoal)
{
    goalDistance = hypot((xGoal - botPositionX), (yGoal - botPositionY));
    x_vec = (xGoal - botPositionX) / goalDistance;
    y_vec = (yGoal - botPositionY) / goalDistance;

    std::vector<int> obsOnRoad;
    point pt;
    for (int i = 0; i < obsticals.size(); i++)
    {
        std::vector<point> pts;
        pts.push_back(pt = {(obsticals[i].x + y_vec * obsticals[i].w), (obsticals[i].y - x_vec * obsticals[i].w), 0.0});
        pts.push_back(pt = {(obsticals[i].x - y_vec * obsticals[i].w), (obsticals[i].y + x_vec * obsticals[i].w), 0.0});
        pts.push_back(pt = {obsticals[i].x, obsticals[i].y, 0.0});
        for (int j = 0; j < 3; j++)
        {
            float x = pts[j].x;
            float y = pts[j].y;
            if (y_vec * y_vec / x_vec * (x - botPositionX) - (y - botPositionY) - R * hypot(x_vec, y_vec) / abs(x_vec) > 0 &&
                y_vec * y_vec / x_vec * (x - botPositionX) - (y - botPositionY) + R * hypot(x_vec, y_vec) / abs(x_vec) < 0 &&
                x_vec * x_vec / -y_vec * (x - botPositionX) - (y - botPositionY) > 0 &&
                x_vec * x_vec / -y_vec * (x - xGoal) - (y - yGoal) < 0)
            {
                obsOnRoad.push_back(i);
                break;
            }
        }
    }
    if (!obsOnRoad.empty())
    {
        hasObs = true;
        planNewPath(obsOnRoad, xGoal, yGoal);
    }

    return;
}

void initParam()
{
    cmd_vel_pub(0, 0, 0);
    xMoved = 0.0, yMoved = 0.0;
    remain = goalDistance;
    // modify velocity params according to different distance
    maxVelocity = max(min(goalDistance / 0.5 * 0.325, 0.4), 0.1);
    vel_0 = 0.05;
    vel_1 = maxVelocity - vel_0;
    vel_2 = maxVelocity;
    dist_0 = 0.05;
    dist_1 = maxVelocity / 1.5 + 0.1 / 3 - dist_0;
    dist_2 = maxVelocity / 1.5 + 0.1 / 3;
}

// TODO: TF !!!
// Transfer the world coordinate into robot coordinate

float TF_World_to_Robot(float World)
{
    float Robot = 0.0;
    Robot = World;
    return Robot;
}

// Return if it's arrived or not
int moveTo()
{
    float VelX, VelY, AngVelW;
    int is_arrived = 0;
    if (abs(remain) > 0.005 /* && abs(lastRemainX) >= abs(remainX)*/)
    {
        xMoved += rVx * deltaTime;
        yMoved += rVy * deltaTime;
        Moved = hypot(xMoved, yMoved);
        remain = goalDistance - Moved;
        if (abs(Moved) <= dist_0)
            VelocityNow = vel_0;
        //            VelocityNow = pow(abs(xMoved) / dist_0, 1.5) * vel_0 ;
        else if (abs(Moved) <= dist_1)
            VelocityNow = (abs(Moved) - dist_0) * 1.5 + vel_0;
        else if (abs(Moved) <= dist_2)
            VelocityNow = pow(((-abs(Moved) + maxVelocity / 1.5 + 0.1 / 3) / dist_0), 1.5) * -vel_0 + maxVelocity;

        else if (abs(remain) <= dist_0)
            VelocityNow = pow(abs(remain) / dist_0, 1.5) * vel_0;
        else if (abs(remain) <= dist_1)
            VelocityNow = (abs(remain) - dist_0) * 1.5 + vel_0;
        else if (abs(remain) <= dist_2)
            VelocityNow = pow(((-abs(remain) + maxVelocity / 1.5 + 0.1 / 3) / dist_0), 1.5) * -vel_0 + maxVelocity;
        else
            VelocityNow = vel_2;

        updateUnitVector(abs(Moved));
        if (goalDistance < 0)
        {
            VelX = -VelocityNow * x_vec;
            VelY = -VelocityNow * y_vec;
        }
        else
        {
            VelX = VelocityNow * x_vec;
            VelY = VelocityNow * y_vec;
        }
        is_arrived = 0;
    }
    else
    {
        VelX = 0;
        VelY = 0;
    }

    if (abs(rW) > 0.00)
        AngVelW = -rW * 0.06;
    else
        AngVelW = 0;

    if (VelX == 0 && VelY == 0)
    {
        botPositionX += xMoved;
        botPositionY += yMoved;
        is_arrived = 1;
    }
    else
        is_arrived = 0;

    // Go through TF
    VelX = TF_World_to_Robot(VelX);
    VelY = TF_World_to_Robot(VelY);
    AngVelW = TF_World_to_Robot(AngVelW);

    // Publish the cmd_vel
    cmd_vel_pub(VelX, VelY, AngVelW);

    // Return the robot status
    return is_arrived;
}
