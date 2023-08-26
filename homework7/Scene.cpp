//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
	//this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::SAH);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
	float emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
		}
	}
	float p = get_random_float() * emit_area_sum;
	emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
			if (p <= emit_area_sum) {
				objects[k]->Sample(pos, pdf);
				break;
			}
		}
	}
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
	// TO DO Implement Path Tracing Algorithm here
	Intersection inter_p = intersect(ray); //从相机到成像平面上某像素的ray和场景交于p点（从0~spp-1、每次肯定都交于同一p、因为ray没变，每次变的是p的出射wo）
	//Vector3f hitColor = this->backgroundColor;
	Vector3f hitColor(0.0);
	//若ray和场景有交点p，才继续
	if (inter_p.happened) {
		Material *material_p = inter_p.m; //交点p处的材质--》为了后面获得p处的brdf
		Vector3f p = inter_p.coords;
		Vector3f n = inter_p.normal; //交点p处的法向量
		Vector3f wo = ray.direction; //相机到成像平面上某一像素的方向
		//---------下面计算p点的全局照明 返回p的shade（wo出射方向）----------
		//1. 直接照明部分
		//sampleLight：在场景的所有光源上按面积 uniform 地 sample 一个点x，并计算该 sample 的概率密度
		Intersection inter_x;
		float pdf_light;
		sampleLight(inter_x, pdf_light);
		Vector3f x = inter_x.coords; //光源上采样点的位置
		Vector3f emit = inter_x.emit; //光源发出的radiance
		Vector3f nn = inter_x.normal; //光源上采样点的法线

		//p到光源上采样到的x建立光线
		Vector3f ws = normalize(x - p);
		//p位置根据物体内外侧加减个偏移量——》没懂？？？？
		Vector3f p_deviation = (dotProduct(ray.direction, n) < 0) ?
			p + n * EPSILON :
			p - n * EPSILON;
		Ray ray_pToLight(p_deviation, ws); //Ray ray_pToLight(p_deviation, ws);

		//若ray_pToLight中间没打到其他物体上--》直接照明
		//（即ray_pToLight和场景求交、如果distance小于|p-x|，证明中间被其他物体挡住了、不是直接照明）精度问题？？
		Intersection inter_temp = intersect(ray_pToLight);
		float bias = 0.01; //0.01
		Vector3f L_dir(0.0);
		float distance = (x - p).norm();
		if (abs(inter_temp.distance - distance) < bias) {
			if (pdf_light < EPSILON) {
				pdf_light = pdf_light + EPSILON;
			}
			L_dir = emit * material_p->eval(wo, ws, n) * dotProduct(ws, n)
				* dotProduct(-ws, nn) / pow(distance, 2) / pdf_light;
		}

		//2. 间接照明部分
		Vector3f L_indir(0.0);
		float ksi = get_random_float();
		if (ksi <= RussianRoulette) {
			//p到物体的连线 和场景求交，如果交点的emit != 0，证明交点是光源、非间接照明、跳过
			Vector3f wi = normalize(material_p->sample(wo, n)); //获得p点有可能打到其他物体的出射方向wi
			Ray ray_pToObject(p_deviation, wi); //注意这里是p_deviation，原点位置变了！ Ray ray_pToObject(p_deviation, wi);
			Intersection inter_object = intersect(ray_pToObject); //刚开始写成这个了ray_pToLight！！
			//如果能wi能打到物体q上，且q不是光源、即无emission
			if (inter_object.happened && !inter_object.m->hasEmission()) {
				//到这里证明交点是物体而非光源、计算间接照明
				//pdf：给定一对入射、出射方向与法向量，计算 sample 方法得到该出射方向的概率密度
				float pdf = material_p->pdf(wo, wi, n);
				//如果pdf接近0，间接照明会很大--》白色噪点？
				if (pdf < EPSILON) {
					pdf = pdf + EPSILON;
				}				
				L_indir = castRay(ray_pToObject, depth + 1) * material_p->eval(wo, wi, n) * dotProduct(wi, n)
					/ pdf / RussianRoulette;			
			}
		}
		//3. 返回全局照明
		hitColor = material_p->getEmission() + L_dir + L_indir;
	}
	//如果ray和场景没交点
	return hitColor;
}