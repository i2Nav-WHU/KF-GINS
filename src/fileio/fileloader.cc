/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
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

#include "fileloader.h"

#include <absl/strings/str_split.h>
#include <iostream>

FileLoader::FileLoader(const string &filename, int columns, int filetype) {
    open(filename, columns, filetype);
}

bool FileLoader::open(const string &filename, int columns, int filetype) {
    auto type = filetype == TEXT ? std::ios_base::in : (std::ios_base::in | std::ios_base::binary);
    filefp_.open(filename, type);

    columns_  = columns;
    filetype_ = filetype;
    return isOpen();
}

vector<double> FileLoader::load() {
    load_();

    return data_;
}

vector<vector<double>> FileLoader::loadn(int epochs) {
    vector<vector<double>> datas;
    datas.clear();

    for (int k = 0; k < epochs; k++) {
        if (load_()) {
            datas.push_back(std::move(data_));
        } else {
            break;
        }
    }

    return datas;
}

bool FileLoader::load(vector<double> &data) {
    if (load_()) {
        data = std::move(data_);
        return true;
    }

    return false;
}

bool FileLoader::loadn(vector<vector<double>> &datas, int epochs) {
    datas.clear();

    for (int k = 0; k < epochs; k++) {
        if (load_()) {
            datas.push_back(std::move(data_));
        } else {
            break;
        }
    }

    return !datas.empty();
}

bool FileLoader::load_() {
    if (isEof())
        return false;

    data_.resize(columns_);

    if (filetype_ == TEXT) {
        string line;
        std::getline(filefp_, line);
        if (line.empty())
            return false;

        vector<string> splits = absl::StrSplit(line, absl::ByAnyChar(", \t"), absl::SkipWhitespace());

        data_.clear();
        for (auto &split : splits) {
            data_.push_back(strtod(split.data(), nullptr));
        }
    } else {
        filefp_.read((char *) data_.data(), sizeof(double) * columns_);
    }

    return true;
}
