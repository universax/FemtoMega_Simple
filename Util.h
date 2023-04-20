#pragma once
/*
 This is utility to that provides converter to convert ob::VideoFrame to cv::Mat.
 cv::Mat mat = ob::get_mat( video_frame );
 Copyright (c) 2023 Tsukasa Sugiura <t.sugiura0204@gmail.com>
 Licensed under the MIT license.
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/

#ifndef __UTIL__
#define __UTIL__

#include <vector>
//#include <limits>

#include <libobsensor/ObSensor.hpp>
#include <opencv2/opencv.hpp>

namespace ob
{
    cv::Mat get_mat(std::shared_ptr<ob::VideoFrame> src, bool deep_copy = true)
    {
        assert(src->dataSize() != 0);

        cv::Mat mat;
        const int32_t width = src->width();
        const int32_t height = src->height();

        const OBFrameType frame_type = src->type();
        const OBFormat format = src->format();

        switch (frame_type)
        {
        case OBFrameType::OB_FRAME_COLOR:
        {
            switch (format)
            {
            case OBFormat::OB_FORMAT_YUYV:
            {
                cv::Mat yuyv = cv::Mat(height, width, CV_8UC2, src->data()).clone();
                cv::cvtColor(yuyv, mat, cv::COLOR_YUV2BGR_YUYV);
                break;
            }
            case OBFormat::OB_FORMAT_YUY2: // not supported by femto mega
            {
                cv::Mat yuy2 = cv::Mat(height, width, CV_8UC2, src->data()).clone();
                cv::cvtColor(yuy2, mat, cv::COLOR_YUV2BGR_YUY2);
                break;
            }
            case OBFormat::OB_FORMAT_UYVY: // not supported by femto mega
            {
                cv::Mat uyvy = cv::Mat(height, width, CV_8UC2, src->data()).clone();
                cv::cvtColor(uyvy, mat, cv::COLOR_YUV2BGR_UYVY);
                break;
            }
            case OBFormat::OB_FORMAT_NV12:
            {
                cv::Mat nv12 = cv::Mat(height + height / 2, width, CV_8UC1, src->data()).clone();
                cv::cvtColor(nv12, mat, cv::COLOR_YUV2BGR_NV12);
                break;
            }
            case OBFormat::OB_FORMAT_NV21: // not supported by femto mega
            {
                cv::Mat nv21 = cv::Mat(height + height / 2, width, CV_8UC1, src->data()).clone();
                cv::cvtColor(nv21, mat, cv::COLOR_YUV2BGR_NV21);
                break;
            }
            case OBFormat::OB_FORMAT_MJPG:
            {
                std::vector<uint8_t> buffer(reinterpret_cast<uint8_t*>(src->data()), reinterpret_cast<uint8_t*>(src->data()) + src->dataSize());
                mat = cv::imdecode(buffer, cv::IMREAD_ANYCOLOR);
                break;
            }
            case OBFormat::OB_FORMAT_H264:
            case OBFormat::OB_FORMAT_H265:
            {
                throw std::runtime_error("[error] not implemented this format!");
                break;
            }
            case OBFormat::OB_FORMAT_GRAY: // not supported by femto mega
            {
                mat = cv::Mat(height, width, CV_8UC1, src->data()).clone();
                break;
            }
            case OBFormat::OB_FORMAT_HEVC:
            {
                throw std::runtime_error("[error] not implemented this format!");
                break;
            }
            case OBFormat::OB_FORMAT_I420: // not supported by femto mega
            {
                cv::Mat i420 = cv::Mat(height, width, CV_8UC2, src->data()).clone();
                cv::cvtColor(i420, mat, cv::COLOR_YUV2BGR_I420);
                break;
            }
            case OBFormat::OB_FORMAT_RGB:
            {
                mat = deep_copy ? cv::Mat(height, width, CV_8UC3, src->data()).clone()
                    : cv::Mat(height, width, CV_8UC3, src->data());
                cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
                break;
            }
            case OBFormat::OB_FORMAT_BGR: // not supported by femto mega
            {
                mat = deep_copy ? cv::Mat(height, width, CV_8UC3, src->data()).clone()
                    : cv::Mat(height, width, CV_8UC3, src->data());
                break;
            }
            case OBFormat::OB_FORMAT_BGRA:
            {
                mat = deep_copy ? cv::Mat(height, width, CV_8UC4, src->data()).clone()
                    : cv::Mat(height, width, CV_8UC4, src->data());
                cv::cvtColor(mat, mat, cv::COLOR_BGRA2BGR);
                break;
            }
            default:
            {
                throw std::runtime_error("[error] failed to convert this format!");
                break;
            }
            }
            break;
        }
        case OBFrameType::OB_FRAME_DEPTH:
        {
            switch (format)
            {
            case OBFormat::OB_FORMAT_Y16:
            case OBFormat::OB_FORMAT_Y10:
            case OBFormat::OB_FORMAT_Y11:
            case OBFormat::OB_FORMAT_Y12:
            case OBFormat::OB_FORMAT_Y14:
            {
                mat = deep_copy ? cv::Mat(height, width, CV_16UC1, reinterpret_cast<uint16_t*>(src->data())).clone()
                    : cv::Mat(height, width, CV_16UC1, reinterpret_cast<uint16_t*>(src->data()));
                break;
            }
            case OBFormat::OB_FORMAT_Y8:
            {
                mat = deep_copy ? cv::Mat(height, width, CV_8UC1, src->data()).clone()
                    : cv::Mat(height, width, CV_8UC1, src->data());
                break;
            }
            default:
            {
                throw std::runtime_error("[error] failed to convert this format!");
                break;
            }
            }
            break;
        }
        case OBFrameType::OB_FRAME_IR:
        {
            switch (format)
            {
            case OBFormat::OB_FORMAT_MJPG:
            {
                // NOTE: this is slower than other formats.
                std::vector<uint8_t> buffer(reinterpret_cast<uint8_t*>(src->data()), reinterpret_cast<uint8_t*>(src->data()) + src->dataSize());
                mat = cv::imdecode(buffer, cv::IMREAD_ANYCOLOR);
                break;
            }
            case OBFormat::OB_FORMAT_Y16:
            {
                mat = deep_copy ? cv::Mat(height, width, CV_16UC1, reinterpret_cast<uint16_t*>(src->data())).clone()
                    : cv::Mat(height, width, CV_16UC1, reinterpret_cast<uint16_t*>(src->data()));
                break;
            }
            case OBFormat::OB_FORMAT_Y8:
            {
                mat = deep_copy ? cv::Mat(height, width, CV_8UC1, src->data()).clone()
                    : cv::Mat(height, width, CV_8UC1, src->data());
                break;
            }
            default:
            {
                throw std::runtime_error("[error] failed to convert this format!");
                break;
            }
            }
            break;
        }
        default:
        {
            throw std::runtime_error("[error] failed to convert this camera type!");
            break;
        }
        }

        return mat;
    }
}

#endif // __UTIL__