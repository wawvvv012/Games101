//
// Created by goksu on 4/6/19.
//

#include <algorithm>
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

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

//原本参数是int；应该改为float；因为+0.5
static bool insideTriangle(float x, float y, const Vector4f* _v){
    Vector3f v[3];
    for(int i=0;i<3;i++)
        v[i] = {_v[i].x(),_v[i].y(), 1.0};
    Vector3f f0,f1,f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Vector3f p(x,y,1.);
    if((p.dot(f0)*f0.dot(v[2])>0) && (p.dot(f1)*f1.dot(v[0])>0) && (p.dot(f2)*f2.dot(v[1])>0))
        return true;
    return false;
}

//计算重心坐标一般表达式中的系数
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (const auto& t:TriangleList)
    {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm {
                (view * model * t->v[0]),
                (view * model * t->v[1]),
                (view * model * t->v[2])
        };

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }

        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {
                inv_trans * to_vec4(t->normal[0], 0.0f),
                inv_trans * to_vec4(t->normal[1], 0.0f),
                inv_trans * to_vec4(t->normal[2], 0.0f)
        };

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);

        // Also pass view space vertice position(vertice vertex:顶点)
		//还没做投影变换，但做了view model 变换的 三角形的三个顶点 viewspace_pos
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) 
{
    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z. 顶点视图空间深度值z
    //    * Z is interpolated view space depth for the current pixel 是当前像素的插值视图空间深度
    //    * zp is depth between zNear and zFar, used for z-buffer zp是zNear和zFar之间的深度，用于z缓冲区

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);

	//1. 首先面被分割成无数个三角形，这个点要在三角形内才能插值属性，所以要先判断点是否在三角形内（作业2：叉乘判断；这里：重心坐标系数计算）（普通判断/MSAA增加采样点判断）
		//先对三角形取包围盒，遍历每个像素判断是否再三角形内。
	//v是数组，存了3个vector4f。注意：这里(x,y,z,1)将w强制变为1！！！导致丢失了w原本记录的三维空间内的深度值（后面需要改正！）
	std::array v = t.toVector4(); 
	float x_min = width, x_max = 0, y_min = height, y_max = 0; //用float是因为下面的floor和ceil返回float
	//遍历每个顶点，得到包围盒
	for (auto vertex : v) {
		x_min = floor(std::min(vertex.x(), x_min)); //向下取整（返回float）
		y_min = floor(std::min(vertex.y(), y_min));
		x_max = ceil(std::max(vertex.x(), x_max)); //向上取整（返回float）
		y_max = ceil(std::max(vertex.y(), y_max));
	}
	//1.1 判断像素是否在三角形内，利用重心坐标
	for (int i = x_min; i < x_max; i++) {
		for (int j = y_min; j < y_max; j++) {
			//注意i和j是像素左上角，要判断(i+0.5,j+0.5)中心是否再三角形内
			if (insideTriangle(i + 0.5, j + 0.5, t.v)) {
				auto[alpha, beta, gamma] = computeBarycentric2D(i + 0.5, j + 0.5, t.v);
				////计算投影空间下的深度z；
				////v[i].w()为实际三维（视图空间）的深度值（注：后面要改！前面tovector函数中强制把w变为1了！！）
				//float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				////属性（深度）插值分子部分
				//float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				////插值的属性
				//zp *= Z;
				////2. 判断：如果像素在前面、是可见的，再做颜色、纹理等插值操作
				//int now_id = get_index(i, j);
				////这里：近大远小（深度zp越大，离视点越远）
				//if (zp < depth_buf[now_id]) {
				//	depth_buf[now_id] = zp;
				//	//法向量插值
				//	//原本是这样写的，编译报错 eigen c2338 this expression is not a lvalue it is read only
				//	//因为auto出来 interpolated_normal 是vector3f类，不能和float的Z相乘
				//	//上面的zp是float是因为对每个顶点（vector4f），都取得.z()来计算
				//	Eigen::Vector3f interpolated_normal = alpha * t.normal[0] / v[0].w() + beta * t.normal[1] / v[1].w() + gamma * t.normal[2] / v[2].w();
				//	//auto interpolated_normal2 = alpha * t.normal[0] / v[0].w() + beta * t.normal[1] / v[1].w() + gamma * t.normal[2] / v[2].w();
				//	interpolated_normal *= Z;
				//	//auto interpolated_normal = interpolate(alpha / v[0].w(), alpha / v[0].w(), alpha / v[0].w(), t.normal[0], t.normal[1], t.normal[2], 1 / Z);
				//	//颜色插值
				//	Eigen::Vector3f interpolated_color = alpha * t.color[0] / v[0].w() + beta * t.color[1] / v[1].w() + gamma * t.color[2] / v[2].w();
				//	interpolated_color *= Z;
				//	//纹理插值
				//	Eigen::Vector2f interpolated_texcoords = alpha * t.tex_coords[0] / v[0].w() + beta * t.tex_coords[1] / v[1].w() + gamma * t.tex_coords[2] / v[2].w();
				//	interpolated_texcoords *= Z;
				//	//底纹插值(？？？没看懂是啥 view_pos是做了model 和 view，但是没做投影变换的三角形的三个顶点)
				//	Eigen::Vector3f interpolated_shadingcoords = alpha * view_pos[0] / v[0].w() + beta * view_pos[1] / v[1].w() + gamma * view_pos[2] / v[2].w();
				//	interpolated_shadingcoords *= Z;

				//	//与作业2不同，不直接将颜色放入frame buffer（可能因为这里属性太多？作业2里只有color）
				//	//3. 插值的结果保存在fragment shader payload这个类里面
				//	//interpolated_normal.normalized()因为法向量要单位向量，所以归一化！
				//	fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
				//	//fragment_shader_payload这个类里面一共五个变量；
				//	//其中四个在构造的时候初始化完了（上面一句），还剩view_pos没初始化
				//	payload.view_pos = interpolated_shadingcoords;
				//	// Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
				//	//rasterize.hpp中通过函数模板定义函数名为fragment_shader的函数，返回Vector3f，输入参数fragment_shader_payload类
				//	auto pixel_color = fragment_shader(payload);
				//	//4. 给该像素上色（函数里把vector3f的color放进frame_buf）
				//	Vector2i pixel{i, j};
				//	set_pixel(pixel, pixel_color);

				//计算投影空间下的深度z；
				//v[i].w()为实际三维（视图空间）的深度值（注：后面要改！前面tovector函数中强制把w变为1了！！）
				float Z = 1.0 / (alpha / view_pos[0].z() + beta / view_pos[1].z() + gamma / view_pos[2].z());
				//属性（深度）插值分子部分
				float zp = alpha * v[0].z() / view_pos[0].z() + beta * v[1].z() / view_pos[1].z() + gamma * v[2].z() / view_pos[2].z();
				//插值的属性
				zp *= Z;
				//2. 判断：如果像素在前面、是可见的，再做颜色、纹理等插值操作
				int now_id = get_index(i, j);
				//这里：近大远小（深度zp越大，离视点越远）
				if (zp < depth_buf[now_id]) {
					depth_buf[now_id] = zp;
					//法向量插值
					//原本是这样写的，编译报错 eigen c2338 this expression is not a lvalue it is read only
					//因为auto出来 interpolated_normal 是vector3f类，不能和float的Z相乘
					//上面的zp是float是因为对每个顶点（vector4f），都取得.z()来计算
					Eigen::Vector3f interpolated_normal = alpha * t.normal[0] / view_pos[0].z() + beta * t.normal[1] / view_pos[1].z() + gamma * t.normal[2] / view_pos[2].z();
					//auto interpolated_normal2 = alpha * t.normal[0] / v[0].w() + beta * t.normal[1] / v[1].w() + gamma * t.normal[2] / v[2].w();
					interpolated_normal *= Z;
					//auto interpolated_normal = interpolate(alpha / v[0].w(), alpha / v[0].w(), alpha / v[0].w(), t.normal[0], t.normal[1], t.normal[2], 1 / Z);
					//颜色插值
					Eigen::Vector3f interpolated_color = alpha * t.color[0] / view_pos[0].z() + beta * t.color[1] / view_pos[1].z() + gamma * t.color[2] / view_pos[2].z();
					interpolated_color *= Z;
					//纹理插值
					Eigen::Vector2f interpolated_texcoords = alpha * t.tex_coords[0] / view_pos[0].z() + beta * t.tex_coords[1] / view_pos[1].z() + gamma * t.tex_coords[2] / view_pos[2].z();
					interpolated_texcoords *= Z;
					//底纹插值(？？？没看懂是啥 view_pos是做了model 和 view，但是没做投影变换的三角形的三个顶点)
					Eigen::Vector3f interpolated_shadingcoords = alpha * view_pos[0] / view_pos[0].z() + beta * view_pos[1] / view_pos[1].z() + gamma * view_pos[2] / view_pos[2].z();
					interpolated_shadingcoords *= Z;

					//与作业2不同，不直接将颜色放入frame buffer（可能因为这里属性太多？作业2里只有color）
					//3. 插值的结果保存在fragment shader payload这个类里面
					//interpolated_normal.normalized()因为法向量要单位向量，所以归一化！
					fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
					//fragment_shader_payload这个类里面一共五个变量；
					//其中四个在构造的时候初始化完了（上面一句），还剩view_pos没初始化
					payload.view_pos = interpolated_shadingcoords;
					// Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
					//rasterize.hpp中通过函数模板定义函数名为fragment_shader的函数，返回Vector3f，输入参数fragment_shader_payload类
					auto pixel_color = fragment_shader(payload);
					//4. 给该像素上色（函数里把vector3f的color放进frame_buf）
					Vector2i pixel{ i, j };
					set_pixel(pixel, pixel_color);
				}	
			}
		}
	}
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

    texture = std::nullopt; //c++17
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}

