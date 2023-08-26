#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>
#include "Sphere.hpp"
// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
    Scene scene(1280, 960);
	//Scene scene(2000, 2000);
    MeshTriangle bunny("./models/bunny/bunny.obj"); //这里面对每个三角形构建BVH 而不是 整个物体
	//MeshTriangle bunny("./models/rock/rock.obj"); //这里面对每个三角形构建BVH 而不是 整个物体

    scene.Add(&bunny); //是meshTriangle，构造函数里会调用new buildBVH()，buildBVH默认值改成了SAH
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 1));
    scene.Add(std::make_unique<Light>(Vector3f(20, 70, 20), 1));

	//Sphere sph1 = Sphere(Vector3f(-0.5, -0.5, 1), 5); //-0.5, -0.5, 1
	//sph1.m = &Material(DIFFUSE_AND_GLOSSY);
	//sph1.m->m_color = Vector3f(0.6, 0.7, 0.8);

	//Sphere sph2 = Sphere(Vector3f(0.5, -0.5, 2), 5);
	//sph2.m->ior = 1.5;
	//sph2.m = &Material(REFLECTION_AND_REFRACTION);

	//scene.Add(&sph1);
	//scene.Add(&sph2);

    scene.buildBVH(); //场景中所有的物体 比如bunny+2个球体

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}