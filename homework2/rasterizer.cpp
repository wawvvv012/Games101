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

//�������const Vector3f* v��ʾ���ǣ���СΪ3��vector v(����v[0]:x y 1)��������ָ�룬û��������������
//ԭ�����Ĳ�����int x, int y, const Vector3f* v��
//�ĳ�float����Ϊ�������x+0.5��y+0.5
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

	//���֮��õ�����{yazb-ybza, zaxb-zbxa, xayb-xbya};��Ϊ������z��0�������ж����һ���Ƿ����0��
	//�������е�����ȡǰ������������ΪEigen���cross()��˺���ֻ����������С��3��ʱ������ã���Remember that cross product is only for vectors of size 3.�� 
	//��eigen��https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html#title6��
	//����ֱ����*��ʾ��ˣ�a * b = xayb-xbya.
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

	//�ҵ�����ǣ�mvp���õ��Ƿ��úõ���������壨�����ת����������ϵ�У����Լ�͸��ͶӰ����
	//m�������������ת����v�����ת����������ϵ�еı任����
	//ֻ�������ϵ�һ���㡢��mvp*�õ㣬�Ϳ��԰ѵ�ͶӰ����Ļ�ϣ�������[-1,1]����Ҫ�任����Ļ�ռ�[0,width]�ȣ�
    Eigen::Matrix4f mvp = projection * view * model; 
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
			//to_vec4�ǰ�vector3f�������һ���������vector4f
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
		//��(x,y,z,w)-->(x/w, y/w, z/w, 1)
		//��Ϊ���� to_vec4���һ��������1.0f������w�Ѿ���1�ˣ����������Ҳ����
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
		//�任����Ļ�ռ䣬�����õ������Ǹı�vert���Ըı�v����
		//����֮ǰx,y,z��Χ��[-1,1]��
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
			//ǰ�涨���ˣ�f1 = (50 - 0.1) / 2.0��f2 = (50 + 0.1) / 2.0;
			//����z = -1 �� z = 1���õ�z�ķ�Χ��[0.1, 50] ��main�ж����znear��zfar
			//����Ҳ��ʹzԽ�����ӵ�ԽԶ
            vert.z() = vert.z() * f1 + f2;
        }

		//�õ�������Ļ�ռ��������ε���������
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
    auto v = t.toVector4(); //v�Ǵ�СΪ3�ġ�ÿ�д�vector4f(x,y,z,1��˳��)������array
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
	
	//����bounding box�����������������Сx ���x ��Сy ���y 
	//ע��v��Ԫ����float����
	float x_min = width, x_max = 0, y_min = height, y_max = 0;
	for (auto v_part : v)
	{
		//floor��ceil��Ϊ�˵õ�����bounding box
		x_min = floor(std::min(v_part.x(), x_min));
		y_min = floor(std::min(v_part.y(), y_min));
		x_max = ceil(std::max(v_part.x(), x_max));
		y_max = ceil(std::max(v_part.y(), y_max));
	}
	//ȡ�� ��Ϊ�Ƕ����ز���������������
	x_min = (int)x_min;
	y_min = (int)y_min;
	x_max = (int)x_max;
	y_max = (int)y_max;

	//�ж�bounding box�ڵ������Ƿ�����������
	//for (int i = x_min; i <= x_max; i++)
	//{
	//	for (int j = y_min; j <= y_max; j++)
	//	{
	//		//+0.5����ȡ�������ģ����½���(0,0)
	//		if (insideTriangle(i+0.5, j+0.5, t.v)) //t.v:����Ϊvector3f��3�����㣻triangle���ж���Ϊvector3f v[3](�ֱ���x y 1 ?)
	//		{
	//			// C++17 �ṹ���󶨣�Ҫ����Ŀ�������c++���Ա�׼ѡ��c++17��׼
	//			//Ҫ��(i+0.5,j+0.5)����������ĵ���z��ֵ����������ĺ���������ҲҪ+0.5
	//			auto[alpha, beta, gamma] = computeBarycentric2D(i+0.5, j+0.5, t.v);
	//			float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w()); //reciprocal����
	//			float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w(); //interpolated ��ֵ
	//			z_interpolated *= w_reciprocal;
	//			
	//			//��ȱȽ�:Ϊ�˷���д���룬���������Խ���ʾ���ӵ�ԽԶ
	//			//����ͬ�� ����-z�������Խ�����ӵ�Խ����
	//			if (z_interpolated < depth_buf[get_index(i,j)])
	//			{
	//				depth_buf[get_index(i, j)] = z_interpolated;
	//				set_pixel(Eigen::Vector3f(i, j, z_interpolated), t.getColor());
	//			}
	//		}
	//	}
	//}

	//super-sampling����anti-aliasing��� MSAA��
	//ԭ���Ǳ���ÿ������ �������ĵ���(i+0.5, j+0.5)
	//���ڶ�ÿ������2*2������ÿ�����ر��ֳ����ĸ������㣬�ֱ��ǣ�(i+0.25, j+0.25) (i+0.25, j+0.75) (i+0.75, j+0.25) (i+0.75, j+0.75)
	for (int i = x_min; i <= x_max; i++)
	{
		for (int j = y_min; j <= y_max; j++)
		{
			int num = 0; //�������� �������η�Χ�ڵĲ���������
			bool flag = 0; //��¼�������Ƿ�ᱻ�ж�Ϊ����������
			std::vector<float> depth_supersample;
			std::vector<Eigen::Vector2f> p{ {float(i + 0.25), float(j + 0.25)}, {float(i + 0.25), float(j + 0.75)}, {float(i + 0.75), float(j + 0.25)}, {float(i + 0.75), float(j + 0.75)} };
			for (int k = 0; k < p.size(); k++) 
			{
				if (insideTriangle(p[k].x(), p[k].y(), t.v))
				{
					flag = 1;
					num++;
					//����ÿ��sample�����
					auto[alpha, beta, gamma] = computeBarycentric2D(p[k].x(), p[k].y(), t.v); //��������������ϵ���������ֵ�������z
					float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
					float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
					z_interpolated *= w_reciprocal;
					////��¼����� (������д����ص����ԣ���Ϊֻ�����zС��ʱ��Ҫsetpixel ����ɫ���������ж�д������ ��ɫ��ֵд�����棬�ᵼ�²�ƥ�䣡)
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