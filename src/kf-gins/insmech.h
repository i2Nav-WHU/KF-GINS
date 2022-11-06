/*
 * KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Liqiang Wang
 *    Contact : wlq@whu.edu.cn
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/types.h"

#include "kf_gins_types.h"

class INSMech {

public:
    /**
     * @brief INS机械编排算法, 利用IMU数据进行速度、位置和姿态更新
     *        INS Mechanization, update velocity, position and attitude using imudata
     * @param [in]     pvapre 上一时刻状态
     *                        the last imustate
     * @param [in,out] pvacur 输出当前时刻状态
     *                        output the current imustate
     * @param [in]     imupre, imucur imudata
     * */
    static void insMech(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);

private:
    /**
     * @breif 位置更新
     *        position update
     * */
    static void posUpdate(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);

    /**
     * @breif 速度更新
     *        velocity update
     * */
    static void velUpdate(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);

    /**
     * @breif 姿态更新
     *        attitude update
     * */
    static void attUpdate(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);
};

