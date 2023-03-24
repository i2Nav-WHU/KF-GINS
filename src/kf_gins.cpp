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

#include <Eigen/Dense>
#include <absl/time/clock.h>
#include <iomanip>
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "common/angle.h"
#include "fileio/filesaver.h"
#include "fileio/gnssfileloader.h"
#include "fileio/imufileloader.h"

#include "kf-gins/gi_engine.h"

bool loadConfig(YAML::Node &config, GINSOptions &options);
void writeNavResult(double time, NavState &navstate, FileSaver &navfile, FileSaver &imuerrfile);
void writeSTD(double time, Eigen::MatrixXd &cov, FileSaver &stdfile);

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cout << "usage: KF-GINS kf-gins.yaml" << std::endl;
        return -1;
    }

    std::cout << std::endl << "KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System" << std::endl << std::endl;
    auto ts = absl::Now();

    // 加载配置文件
    // load configuration file
    YAML::Node config;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to read configuration file. Please check the path and format of the configuration file!"
                  << std::endl;
        return -1;
    }

    // 读取配置参数到GINSOptioins中，并构造GIEngine
    // load configuration parameters to GINSOptioins
    GINSOptions options;
    if (!loadConfig(config, options)) {
        std::cout << "Error occurs in the configuration file!" << std::endl;
        return -1;
    }

    // 读取文件路径配置
    // load filepath configuration
    std::string imupath, gnsspath, outputpath;
    try {
        imupath    = config["imupath"].as<std::string>();
        gnsspath   = config["gnsspath"].as<std::string>();
        outputpath = config["outputpath"].as<std::string>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check the file path and output path!" << std::endl;
        return -1;
    }

    // imu数据配置，数据处理区间
    // imudata configuration， data processing interval
    int imudatalen, imudatarate;
    double starttime, endtime;
    try {
        imudatalen  = config["imudatalen"].as<int>();
        imudatarate = config["imudatarate"].as<int>();
        starttime   = config["starttime"].as<double>();
        endtime     = config["endtime"].as<double>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check the data length, data rate, and the process time!"
                  << std::endl;
        return -1;
    }

    // 加载GNSS文件和IMU文件
    // load GNSS file and IMU file
    GnssFileLoader gnssfile(gnsspath);
    ImuFileLoader imufile(imupath, imudatalen, imudatarate);

    // 构造GIEngine
    // Construct GIEngine
    GIEngine giengine(options);

    // 构造输出文件
    // construct output file
    // navfile: gnssweek(1) + time(1) + pos(3) + vel(3) + euler angle(3) = 11
    // imuerrfile: time(1) + gyrbias(3) + accbias(3) + gyrscale(3) + accscale(3) = 13
    // stdfile: time(1) + pva_std(9) + imubias_std(6) + imuscale_std(6) = 22
    int nav_columns = 11, imuerr_columns = 13, std_columns = 22;
    FileSaver navfile(outputpath + "/KF_GINS_Navresult.nav", nav_columns, FileSaver::TEXT);
    FileSaver imuerrfile(outputpath + "/KF_GINS_IMU_ERR.txt", imuerr_columns, FileSaver::TEXT);
    FileSaver stdfile(outputpath + "/KF_GINS_STD.txt", std_columns, FileSaver::TEXT);

    // 检查文件是否正确打开
    // check if these files are all opened
    if (!gnssfile.isOpen() || !imufile.isOpen() || !navfile.isOpen() || !imuerrfile.isOpen() || !stdfile.isOpen()) {
        std::cout << "Failed to open data file!" << std::endl;
        return -1;
    }

    // 检查处理时间
    // check process time
    if (endtime < 0) {
        endtime = imufile.endtime();
    }
    if (endtime > 604800 || starttime < imufile.starttime() || starttime > endtime) {
        std::cout << "Process time ERROR!" << std::endl;
        return -1;
    }

    // 数据对齐
    // data alignment
    IMU imu_cur;
    do {
        imu_cur = imufile.next();
    } while (imu_cur.time < starttime);

    GNSS gnss;
    do {
        gnss = gnssfile.next();
    } while (gnss.time <= starttime);

    // 添加IMU数据到GIEngine中，补偿IMU误差
    // add imudata to GIEngine and compensate IMU error
    giengine.addImuData(imu_cur, true);

    // 添加GNSS数据到GIEngine
    // add gnssdata to GIEngine
    giengine.addGnssData(gnss);

    // 用于保存处理结果
    // used to save processing results
    double timestamp;
    NavState navstate;
    Eigen::MatrixXd cov;

    // 用于显示处理进程
    // used to display processing progress
    int percent = 0, lastpercent = 0;
    double interval = endtime - starttime;

    while (true) {
        // 当前IMU状态时间新于GNSS时间时，读取并添加新的GNSS数据到GIEngine
        // load new gnssdata when current state time is newer than GNSS time and add it to GIEngine
        if (gnss.time < imu_cur.time && !gnssfile.isEof()) {
            gnss = gnssfile.next();
            giengine.addGnssData(gnss);
        }

        // 读取并添加新的IMU数据到GIEngine
        // load new imudata and add it to GIEngine
        imu_cur = imufile.next();
        if (imu_cur.time > endtime || imufile.isEof()) {
            break;
        }
        giengine.addImuData(imu_cur);

        // 处理新的IMU数据
        // process new imudata
        giengine.newImuProcess();

        // 获取当前时间，IMU状态和协方差
        // get current timestamp, navigation state and covariance
        timestamp = giengine.timestamp();
        navstate  = giengine.getNavState();
        cov       = giengine.getCovariance();

        // 保存处理结果
        // save processing results
        writeNavResult(timestamp, navstate, navfile, imuerrfile);
        writeSTD(timestamp, cov, stdfile);

        // 显示处理进展
        // display processing progress
        percent = int((imu_cur.time - starttime) / interval * 100);
        if (percent - lastpercent >= 1) {
            std::cout << " - Processing: " << std::setw(3) << percent << "%\r" << std::flush;
            lastpercent = percent;
        }
    }

    // 关闭打开的文件
    // close opened file
    imufile.close();
    gnssfile.close();
    navfile.close();
    imuerrfile.close();
    stdfile.close();

    // 处理完毕
    // process finish
    auto te = absl::Now();
    std::cout << std::endl << std::endl << "KF-GINS Process Finish! ";
    std::cout << "From " << starttime << " s to " << endtime << " s, total " << interval << " s!" << std::endl;
    std::cout << "Cost " << absl::ToDoubleSeconds(te - ts) << " s in total" << std::endl;

    return 0;
}

/**
 * @brief 从配置文件中读取GIEngine相关的初始状态，并转换为标准单位
 *        Load initial states of GIEngine from configuration file and convert them to standard units
 * */
bool loadConfig(YAML::Node &config, GINSOptions &options) {

    // 读取初始位置(纬度 经度 高程)、(北向速度 东向速度 垂向速度)、姿态(欧拉角，ZYX旋转顺序, 横滚角、俯仰角、航向角)
    // load initial position(latitude longitude altitude)
    //              velocity(speeds in the directions of north, east and down)
    //              attitude(euler angle, ZYX, roll, pitch and yaw)
    std::vector<double> vec1, vec2, vec3, vec4, vec5, vec6;
    try {
        vec1 = config["initpos"].as<std::vector<double>>();
        vec2 = config["initvel"].as<std::vector<double>>();
        vec3 = config["initatt"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check initial position, velocity, and attitude!"
                  << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        options.initstate.pos[i]   = vec1[i] * D2R;
        options.initstate.vel[i]   = vec2[i];
        options.initstate.euler[i] = vec3[i] * D2R;
    }
    options.initstate.pos[2] *= R2D;

    // 读取IMU误差初始值(零偏和比例因子)
    // load initial imu error (bias and scale factor)
    try {
        vec1 = config["initgyrbias"].as<std::vector<double>>();
        vec2 = config["initaccbias"].as<std::vector<double>>();
        vec3 = config["initgyrscale"].as<std::vector<double>>();
        vec4 = config["initaccscale"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check initial IMU error!" << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        options.initstate.imuerror.gyrbias[i]  = vec1[i] * D2R / 3600.0;
        options.initstate.imuerror.accbias[i]  = vec2[i] * 1e-5;
        options.initstate.imuerror.gyrscale[i] = vec3[i] * 1e-6;
        options.initstate.imuerror.accscale[i] = vec4[i] * 1e-6;
    }

    // 读取初始位置、速度、姿态(欧拉角)的标准差
    // load initial position std, velocity std and attitude(euler angle) std
    try {
        vec1 = config["initposstd"].as<std::vector<double>>();
        vec2 = config["initvelstd"].as<std::vector<double>>();
        vec3 = config["initattstd"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check initial std of position, velocity, and attitude!"
                  << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        options.initstate_std.pos[i]   = vec1[i];
        options.initstate_std.vel[i]   = vec2[i];
        options.initstate_std.euler[i] = vec3[i] * D2R;
    }

    // 读取IMU噪声参数
    // load imu noise parameters
    try {
        vec1 = config["imunoise"]["arw"].as<std::vector<double>>();
        vec2 = config["imunoise"]["vrw"].as<std::vector<double>>();
        vec3 = config["imunoise"]["gbstd"].as<std::vector<double>>();
        vec4 = config["imunoise"]["abstd"].as<std::vector<double>>();
        vec5 = config["imunoise"]["gsstd"].as<std::vector<double>>();
        vec6 = config["imunoise"]["asstd"].as<std::vector<double>>();

        options.imunoise.corr_time = config["imunoise"]["corrtime"].as<double>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check IMU noise!" << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        options.imunoise.gyr_arw[i]      = vec1[i];
        options.imunoise.acc_vrw[i]      = vec2[i];
        options.imunoise.gyrbias_std[i]  = vec3[i];
        options.imunoise.accbias_std[i]  = vec4[i];
        options.imunoise.gyrscale_std[i] = vec5[i];
        options.imunoise.accscale_std[i] = vec6[i];
    }

    // 读取IMU误差初始标准差,如果配置文件中没有设置，则采用IMU噪声参数中的零偏和比例因子的标准差
    // Load initial imu bias and scale std, set to bias and scale instability std if load failed
    try {
        vec1 = config["initbgstd"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        vec1 = {options.imunoise.gyrbias_std.x(), options.imunoise.gyrbias_std.y(), options.imunoise.gyrbias_std.z()};
    }
    try {
        vec2 = config["initbastd"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        vec2 = {options.imunoise.accbias_std.x(), options.imunoise.accbias_std.y(), options.imunoise.accbias_std.z()};
    }
    try {
        vec3 = config["initsgstd"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        vec3 = {options.imunoise.gyrscale_std.x(), options.imunoise.gyrscale_std.y(),
                options.imunoise.gyrscale_std.z()};
    }
    try {
        vec4 = config["initsastd"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        vec4 = {options.imunoise.accscale_std.x(), options.imunoise.accscale_std.y(),
                options.imunoise.accscale_std.z()};
    }
    // IMU初始误差转换为标准单位
    // convert initial imu errors' units to standard units
    for (int i = 0; i < 3; i++) {
        options.initstate_std.imuerror.gyrbias[i]  = vec1[i] * D2R / 3600.0;
        options.initstate_std.imuerror.accbias[i]  = vec2[i] * 1e-5;
        options.initstate_std.imuerror.gyrscale[i] = vec3[i] * 1e-6;
        options.initstate_std.imuerror.accscale[i] = vec4[i] * 1e-6;
    }

    // IMU噪声参数转换为标准单位
    // convert imu noise parameters' units to standard units
    options.imunoise.gyr_arw *= (D2R / 60.0);
    options.imunoise.acc_vrw /= 60.0;
    options.imunoise.gyrbias_std *= (D2R / 3600.0);
    options.imunoise.accbias_std *= 1e-5;
    options.imunoise.gyrscale_std *= 1e-6;
    options.imunoise.accscale_std *= 1e-6;
    options.imunoise.corr_time *= 3600;

    // GNSS天线杆臂, GNSS天线相位中心在IMU坐标系下位置
    // gnss antenna leverarm, position of GNSS antenna phase center in IMU frame
    try {
        vec1 = config["antlever"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check antenna leverarm!" << std::endl;
        return false;
    }
    options.antlever = Eigen::Vector3d(vec1.data());

    return true;
}

/**
 * @brief 保存导航结果和IMU误差，已转换为常用单位
 *        save navigation result and imu error, converted them to common units
 * */
void writeNavResult(double time, NavState &navstate, FileSaver &navfile, FileSaver &imuerrfile) {

    std::vector<double> result;

    // 保存导航结果
    // save navigation result
    result.clear();
    result.push_back(0);
    result.push_back(time);
    result.push_back(navstate.pos[0] * R2D);
    result.push_back(navstate.pos[1] * R2D);
    result.push_back(navstate.pos[2]);
    result.push_back(navstate.vel[0]);
    result.push_back(navstate.vel[1]);
    result.push_back(navstate.vel[2]);
    result.push_back(navstate.euler[0] * R2D);
    result.push_back(navstate.euler[1] * R2D);
    result.push_back(navstate.euler[2] * R2D);
    navfile.dump(result);

    // 保存IMU误差
    // save IMU error
    auto imuerr = navstate.imuerror;
    result.clear();
    result.push_back(time);
    result.push_back(imuerr.gyrbias[0] * R2D * 3600);
    result.push_back(imuerr.gyrbias[1] * R2D * 3600);
    result.push_back(imuerr.gyrbias[2] * R2D * 3600);
    result.push_back(imuerr.accbias[0] * 1e5);
    result.push_back(imuerr.accbias[1] * 1e5);
    result.push_back(imuerr.accbias[2] * 1e5);
    result.push_back(imuerr.gyrscale[0] * 1e6);
    result.push_back(imuerr.gyrscale[1] * 1e6);
    result.push_back(imuerr.gyrscale[2] * 1e6);
    result.push_back(imuerr.accscale[0] * 1e6);
    result.push_back(imuerr.accscale[1] * 1e6);
    result.push_back(imuerr.accscale[2] * 1e6);
    imuerrfile.dump(result);
}

/**
 * @brief 保存标准差，已转换为常用单位
 *        save standard deviation, converted to common units
 * */
void writeSTD(double time, Eigen::MatrixXd &cov, FileSaver &stdfile) {

    std::vector<double> result;

    result.clear();
    result.push_back(time);
    // 保存位置、速度、姿态标准差
    // save position, velocity and attitude std
    for (int i = 0; i < 6; i++) {
        result.push_back(sqrt(cov(i, i)));
    }
    for (int i = 6; i < 9; i++) {
        result.push_back(sqrt(cov(i, i)) * R2D);
    }

    // 保存IMU误差标准差
    // save imu error std
    for (int i = 9; i < 12; i++) {
        result.push_back(sqrt(cov(i, i)) * R2D * 3600);
    }
    for (int i = 12; i < 15; i++) {
        result.push_back(sqrt(cov(i, i)) * 1e5);
    }
    for (int i = 15; i < 21; i++) {
        result.push_back(sqrt(cov(i, i)) * 1e6);
    }
    stdfile.dump(result);
}