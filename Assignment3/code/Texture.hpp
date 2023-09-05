//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color00 = image_data.at<cv::Vec3b>(int(v_img), int(u_img));
        auto color01 = image_data.at<cv::Vec3b>(int(v_img), int(u_img) + 1);
        auto color10 = image_data.at<cv::Vec3b>(int(v_img) + 1, int(u_img));
        auto color11 = image_data.at<cv::Vec3b>(int(v_img) + 1, int(u_img) + 1);
        float s = v_img - int(v_img);
        float t = u_img - int(u_img);
        auto color00f = Eigen::Vector3f(color00[0], color00[1], color00[2]);
        auto color01f = Eigen::Vector3f(color01[0], color01[1], color01[2]);
        auto color10f = Eigen::Vector3f(color10[0], color10[1], color10[2]);
        auto color11f = Eigen::Vector3f(color11[0], color11[1], color11[2]);
        return color00f * (1 - s) * (1 - t)
            +  color01f * (1 - s) * t
            +  color10f * s * (1 - t)
            +  color11f * s * t;
    }
};
#endif //RASTERIZER_TEXTURE_H
