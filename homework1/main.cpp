#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

//define是预编译阶段进行字符串模板的替换；代码不会进行类型检查：符号丢失、没有范围、没有类型
//constexpr是编译阶段；在编译期把结果计算出来并替换（在函数内部定义时，是运行阶段计算结果并赋值）
//const是编译阶段：在运行阶段可以保证常量是只读的、不能改变。
constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float angle = rotation_angle * MY_PI / 180;
    model << cos(angle), -sin(angle), 0, 0, 
            sin(angle), cos(angle), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
	std::cout << "model's rotation:" << std::endl;
    std::cout << model << std::endl;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    //初始化为单位阵
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    //假设t=-b l=-r，所以正交投影矩阵Mo是对称的（除了关于xy平面，因为znear和zfar不一定互为相反数）
    float fov = eye_fov * MY_PI / 180;
    float t = abs(zNear) * tan(fov / 2);
	float b = -t;
    float r = aspect_ratio * t;
	float l = -r;
    Eigen::Matrix4f Mo_rotate = Eigen::Matrix4f::Identity(); //正交投影旋转矩阵
    if (zFar != -zNear) 
    {
        Mo_rotate << 2 / (r - l), 0, 0, 0,
              0, 2 / (t - b), 0, 0,
              0, 0, 2 / (zNear + zFar), 0,
              0, 0, 0, 1;
    }
    //避免出现分母为0
    else
    {
        Mo_rotate << 2 /(r-l), 0, 0, 0,
					 0, 2 /(t-b), 0, 0,
					 0, 0, 2 / (zNear - zFar), 0,
					 0, 0, 0, 1;
    }
	Eigen::Matrix4f Mo_trans = Eigen::Matrix4f::Identity(); //正交投影平移矩阵
	Mo_trans << 1, 0, 0, -(r + l) / 2,
				0, 1, 0, -(t + b) / 2,
				0, 0, 1, -(zNear + zFar) / 2,
				0, 0, 0, 1;
	Eigen::Matrix4f Mo = Eigen::Matrix4f::Identity(); //正交投影矩阵
	Mo = Mo_rotate * Mo_trans; //之所以不直接直接把旋转平移写到一个矩阵Mo中，是因为这样的矩阵Mo会和待变换的点，先计算旋转（线性变换）、后计算平移

    Eigen::Matrix4f Mp2o = Eigen::Matrix4f::Identity(); //透视投影的frustum--》挤压成正交投影的cuboid
    Mp2o << zNear, 0, 0, 0, 
            0, zNear, 0, 0,
            0, 0, zNear + zFar, -zNear * zFar,
            0, 0, 1, 0;
    
    projection = Mo * Mp2o; //先变成正交投影的cuboid再正交投影

    return projection;
}

////在 main.cpp 中构造一个函数，该函数的作用是得到绕任意过原点的轴的旋转变换矩阵。
////罗德里格斯旋转矩阵
Eigen::Matrix3f get_rotation(Vector3f axis, float angle)
{
	float rotation_angle = angle * MY_PI / 180;
	Eigen::Matrix3f I = Eigen::Matrix3f::Identity(); //构造单位阵
	Eigen::Matrix3f axis_matrix = Eigen::Matrix3f::Identity(); //构造轴的矩阵形式
	axis_matrix << 0, -axis[2], axis[1],
		axis[2], 0, -axis[0],
		-axis[1], axis[0], 0;

	Eigen::Matrix3f rotation = cos(rotation_angle) * I + (1 - cos(rotation_angle)) * axis * axis.transpose() + sin(rotation_angle) * axis_matrix;
	std::cout << "Rodrigues' Rotation:" << std::endl;
	std::cout << rotation << std::endl;
	return rotation;
}
int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50)); //如果改成负的，就是正的三角形了，否则是倒的。因为这里相机是看向-z方向，znear和zfar默认为负

		Eigen::Vector3f axis(0.0f, 0.0f, 1.0f);
		get_rotation(axis, angle);

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
