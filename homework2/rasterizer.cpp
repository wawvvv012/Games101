// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//这里参数const Vector3f* v表示的是：大小为3的vector v(比如v[0]:x y 1)，而不是指针，没看懂？？？？？
//原本给的参数是int x, int y, const Vector3f* v；
//改成float，因为传入的是x+0.5，y+0.5
static bool insideTriangle(float x, float y, const Vector3f* v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
	//Eigen::Vector3f point = { x, y, 1.0f };
	Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
	Eigen::Vector3f a = v[0] - v[1];
	Eigen::Vector3f b = v[1] - v[2];
	Eigen::Vector3f c = v[2] - v[0];

	Eigen::Vector3f v0p = v[0] - point;
	Eigen::Vector3f v1p = v[1] - point;
	Eigen::Vector3f v2p = v[2] - point;

	//叉乘之后得到向量{yazb-ybza, zaxb-zbxa, xayb-xbya};因为向量的z是0，所以判断最后一项是否大于0；
	//或者所有的向量取前两个，但是因为Eigen库的cross()叉乘函数只有在向量大小是3的时候才能用：“Remember that cross product is only for vectors of size 3.” 
	//（eigen：https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html#title6）
	//所以直接用*表示叉乘，a * b = xayb-xbya.
	if ((a.cross(v0p).z() < 0 && b.cross(v1p).z() < 0 && c.cross(v2p).z() < 0) || 
		(a.cross(v0p).z() > 0 && b.cross(v1p).z() > 0 && c.cross(v2p).z() > 0))
	{
		return true;
	}
	else
	{
		return false;
	}
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

	//我的理解是：mvp后获得的是放置好的相机、物体（把相机转到物体坐标系中）、以及透视投影矩阵；
	//m：物体自身的旋转矩阵；v：相机转到物体坐标系中的变换矩阵；
	//只差物体上的一个点、用mvp*该点，就可以把点投影到屏幕上（物体是[-1,1]，还要变换到屏幕空间[0,width]等）
    Eigen::Matrix4f mvp = projection * view * model; 
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
			//to_vec4是把vector3f加上最后一个参数变成vector4f
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
		//把(x,y,z,w)-->(x/w, y/w, z/w, 1)
		//因为上面 to_vec4最后一个参数是1.0f，所以w已经是1了，不做下面的也可以
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
		//变换到屏幕空间，用引用的意义是改变vert可以改变v本身
		//（便之前x,y,z范围：[-1,1]）
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
			//前面定义了：f1 = (50 - 0.1) / 2.0；f2 = (50 + 0.1) / 2.0;
			//带入z = -1 和 z = 1，得到z的范围：[0.1, 50] 即main中定义的znear和zfar
			//而且也会使z越大、离视点越远
            vert.z() = vert.z() * f1 + f2;
        }

		//得到最终屏幕空间中三角形的三个顶点
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4(); //v是大小为3的、每行存vector4f(x,y,z,1的顺序)的数组array
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
	
	//创建bounding box：获得三个顶点中最小x 最大x 最小y 最大y 
	//注意v中元素是float类型
	float x_min = width, x_max = 0, y_min = height, y_max = 0;
	for (auto v_part : v)
	{
		//floor和ceil是为了得到最大的bounding box
		x_min = floor(std::min(v_part.x(), x_min));
		y_min = floor(std::min(v_part.y(), y_min));
		x_max = ceil(std::max(v_part.x(), x_max));
		y_max = ceil(std::max(v_part.y(), y_max));
	}
	//取整 因为是对像素操作，像素是整数
	x_min = (int)x_min;
	y_min = (int)y_min;
	x_max = (int)x_max;
	y_max = (int)y_max;

	//判断bounding box内的像素是否再三角形内
	//for (int i = x_min; i <= x_max; i++)
	//{
	//	for (int j = y_min; j <= y_max; j++)
	//	{
	//		//+0.5：获取像素中心；左下角是(0,0)
	//		if (insideTriangle(i+0.5, j+0.5, t.v)) //t.v:类型为vector3f的3个顶点；triangle类中定义为vector3f v[3](分别是x y 1 ?)
	//		{
	//			// C++17 结构化绑定：要把项目里的属性c++语言标准选成c++17标准
	//			//要对(i+0.5,j+0.5)这个像素中心点做z插值，所以下面的函数参数中也要+0.5
	//			auto[alpha, beta, gamma] = computeBarycentric2D(i+0.5, j+0.5, t.v);
	//			float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w()); //reciprocal倒数
	//			float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w(); //interpolated 插值
	//			z_interpolated *= w_reciprocal;
	//			
	//			//深度比较:为了方便写代码，这里是深度越大表示离视点越远
	//			//（不同于 看向-z方向，深度越大离视点越近）
	//			if (z_interpolated < depth_buf[get_index(i,j)])
	//			{
	//				depth_buf[get_index(i, j)] = z_interpolated;
	//				set_pixel(Eigen::Vector3f(i, j, z_interpolated), t.getColor());
	//			}
	//		}
	//	}
	//}

	//super-sampling处理anti-aliasing锯齿 MSAA：
	//原本是遍历每个像素 像素中心点是(i+0.5, j+0.5)
	//现在对每个像素2*2采样，每个像素被分成了四个采样点，分别是：(i+0.25, j+0.25) (i+0.25, j+0.75) (i+0.75, j+0.25) (i+0.75, j+0.75)
	for (int i = x_min; i <= x_max; i++)
	{
		for (int j = y_min; j <= y_max; j++)
		{
			int num = 0; //超采样中 在三角形范围内的采样点数量
			bool flag = 0; //记录整像素是否会被判断为在三角形内
			std::vector<float> depth_supersample;
			std::vector<Eigen::Vector2f> p{ {float(i + 0.25), float(j + 0.25)}, {float(i + 0.25), float(j + 0.75)}, {float(i + 0.75), float(j + 0.25)}, {float(i + 0.75), float(j + 0.75)} };
			for (int k = 0; k < p.size(); k++) 
			{
				if (insideTriangle(p[k].x(), p[k].y(), t.v))
				{
					flag = 1;
					num++;
					//计算每个sample的深度
					auto[alpha, beta, gamma] = computeBarycentric2D(p[k].x(), p[k].y(), t.v); //根据这个算出来的系数，下面插值计算深度z
					float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
					float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
					z_interpolated *= w_reciprocal;
					////记录最近的 (在这里写最后重叠不对！因为只有深度z小的时候要setpixel 赋颜色，如果深度判断写在这里 颜色赋值写在外面，会导致不匹配！)
					//if (z_interpolated < depth_buf[get_index(i, j)])
					//{
					//	depth_buf[get_index(i, j)] = z_interpolated;
					//}
					depth_supersample.push_back(z_interpolated);
				}
			}
			if (flag)
			{
				assert(num <= 4);
				for (int k = 0; k < depth_supersample.size(); k++)
				{
					if (depth_supersample[k] < depth_buf[get_index(i, j)])
					{
						depth_buf[get_index(i, j)] = depth_supersample[k];
						set_pixel(Eigen::Vector3f(i, j, 0), t.getColor() * num / 4);
					}
				}
				
			}
			
		}
	}
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on