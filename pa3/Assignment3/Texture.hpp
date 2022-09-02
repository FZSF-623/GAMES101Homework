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

    int rangeSafe(int x, bool isU)
    {
        if(x<0) return 0;

        if(isU&&x>=width) return width-1;
        if(!isU&&x>=height) return height-1;

        return x;
    }

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
        u = std::max(0.0f,u) ,u = std::min(1.0f,u);
        v = std::max(0.0f,v) ,v = std::min(1.0f,v);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u,float v){
        float u_img = u * width;
        float v_img = (1 - v) * height;

        int u_min = rangeSafe(floor(u_img),true);
        int u_max = rangeSafe(ceil(u_img),true);
        int v_min = rangeSafe(floor(v_img),true);
        int v_max = rangeSafe(ceil(v_img),true);

        auto U00 = image_data.at<cv::Vec3b>(v_max, u_min);
        auto U10 = image_data.at<cv::Vec3b>(v_max, u_max);
        auto U01 = image_data.at<cv::Vec3b>(v_min, u_min);
        auto U11 = image_data.at<cv::Vec3b>(v_min, u_max);

        float lerp_s = (u_img - u_min) / (u_max - u_min);
        float lerp_t = (v_img - v_min) / (v_max - v_min);

        auto cTop = (1-lerp_s) * U01 + lerp_s * U11;
        auto cBot = (1-lerp_s) * U00 + lerp_s * U10;

        auto P = (1 - lerp_t) * cTop + lerp_t * cBot;
        return Eigen::Vector3f(P[0], P[1], P[2]);
        
    }

};
#endif //RASTERIZER_TEXTURE_H
