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
		if (image_data.empty())
		{
			std::cout << "纹理图片读入失败" << std::endl;
		}
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

	//查找纹理颜色接口
    Eigen::Vector3f getColor(float u, float v)
    {
		// 坐标限定,后加，不然会有
		//Error: Assertion failed ((unsigned)(i1 * DataType<_Tp>::channels) < (unsigned)(size.p[1] * channels())) 
		//in cv::Mat的异常
		if (u < 0) u = 0;
		if (u > 1) u = 1;
		if (v < 0) v = 0;
		if (v > 1) v = 1;

        auto u_img = u * width;
		//1-v是因为Mat中(0,0)是左上角；纹理坐标中(0,0)是左下角
        auto v_img = (1 - v) * height;
		//vec3b:放了3个uchar的数组，uchar 0-255
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

	//双线性插值（若纹理太小，多个像素对应同一个纹素）
	Eigen::Vector3f getColorBilinear(float u, float v) {

		//原本u v在 [0,1]，这里变成[0,width] [0,height]
		u = u * width;
		v = (1-v) * height;
		float u0 = round(u) - 1; 
		float u1 = round(u);
		float v0 = round(v) - 1;
		float v1 = round(v);

		if (u0 < 0) u0 = 0;
		if (u0 >= width) u0 = width - 1;
		if (u1 < 0) u1 = 0;
		if (u1 >= width) u1 = width - 1;
		if (v0 < 0) v0 = 0;
		if (v0 >= height) v0 = height - 1;
		if (v1 < 0) v1 = 0;
		if (v1 >= height) v1 = height - 1;

		//vec3b:放了3个uchar的数组，uchar 0-255
		auto color00 = image_data.at<cv::Vec3b>(v0, u0);
		auto color01 = image_data.at<cv::Vec3b>(v1, u0);
		auto color10 = image_data.at<cv::Vec3b>(v0, u1);
		auto color11 = image_data.at<cv::Vec3b>(v1, u1);
		//尺度因子
		float s = (u - u0) / (u1 - u0);
		float t = (v - v0) / (v1 - v0);
		//双线性插值
		auto color0 = color00 + s * (color01 - color00);
		auto color1 = color10 + s * (color11 - color10);

		auto color = color0 + t * (color1 - color0);
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}
};
#endif //RASTERIZER_TEXTURE_H
