//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include <mutex>          // std::mutex

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
	std::vector<Vector3f> framebuffer(scene.width * scene.height);

	float scale = tan(deg2rad(scene.fov * 0.5));
	float imageAspectRatio = scene.width / (float)scene.height;
	Vector3f eye_pos(278, 273, -800);
	int m = 0;

	// change the spp value to change sample ammount
	int spp = 512;//16
	int thread_num = 14;//我的电脑有8核，所以开8个线程。注：屏幕的高度一定要是线程数的倍数
	int thread_height = scene.height / thread_num; //每个线程处理width * thread_height这么多个像素着色
	std::vector<std::thread> threads(thread_num);
	std::cout << "SPP: " << spp << "\n";

	//多线程实现
	std::mutex mtx;
	float process = 0;
	float Reciprocal_Scene_height = 1.f / (float)scene.height;
	//&表示能用castRay函数里面能用函数外的所有变量
	auto castRay = [&](int thread_index)
	{
		int height = thread_height * (thread_index + 1);
		for (uint32_t j = height - thread_height; j < height; j++)
		{
			for (uint32_t i = 0; i < scene.width; ++i) {
				// generate primary ray direction
				float x = (2 * (i + 0.5) / (float)scene.width - 1) *
					imageAspectRatio * scale;
				float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

				//eye的位置对结果有影响
				Vector3f dir = normalize(Vector3f(-x, y, 1));
				for (int k = 0; k < spp; k++)
				{
					framebuffer[j*scene.width + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
				}
			}
			mtx.lock(); //mtx互斥锁：lock和unlock中间的部分、一个进程在用的时候不允许其他进程用（避免控制台显示的进度条混乱）
			process = process + Reciprocal_Scene_height;
			UpdateProgress(process); //cout进度条
			mtx.unlock();
		}
	};

	for (int k = 0; k < thread_num; k++)
	{
		threads[k] = std::thread(castRay, k); //利用castRay这个lambda函数，依次开启第k个线程
	}
	for (int k = 0; k < thread_num; k++)
	{
		threads[k].join(); //线程添加方式为join（等所有k个线程执行结束才能执行后面的主程序）
	}
	UpdateProgress(1.f);

	// save framebuffer to file
	FILE* fp = fopen("binary.ppm", "wb");
	(void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
	for (auto i = 0; i < scene.height * scene.width; ++i) {
		static unsigned char color[3];
		color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
		color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
		color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
		fwrite(color, 1, 3, fp);
	}
	fclose(fp);
}


