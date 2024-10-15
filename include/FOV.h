/**
* This file is part of https://github.com/JingwenWang95/DSP-SLAM
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#ifndef FOV_H
#define FOV_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "core/Plane.h"


class FOV {
public:
    FOV();

    // cuboid
    float w;
    float h;
    float l;

    Cuboid3D mCuboid3D;                 // cuboid.

    void Update_Twobj();                // 原UpdateObjPose  更新物体在世界下的坐标
    void Update_Twobj(double x, double y, double z, double yaw);                // 原UpdateObjPose  更新物体在世界下的坐标
    void Update_object_size(double lenth, double width, double height)   ;   //更新物体的尺寸

};


#endif //MAPOBJECT_H
