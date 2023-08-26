#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

	if (splitMethod == SplitMethod::SAH) {
		root = recursiveSAHBuild(primitives);
	}
	else if (splitMethod == SplitMethod::NAIVE) {
		root = recursiveBuild(primitives);
	}

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }

    return node;
}

// SAH 表面积启发式算法
BVHBuildNode* BVHAccel::recursiveSAHBuild(std::vector<Object*> objects)
{
	BVHBuildNode* node = new BVHBuildNode();

	// Compute bounds of all primitives in BVH node
	Bounds3 bounds;
	//获取场景中所有物体的最大包围盒
	for (int i = 0; i < objects.size(); ++i)
		//Bounds3::Union 函数的作用是将两个包围盒并成更大的包围盒。与材质一样，场景中的每个物体实例都有自己的包围盒。
		bounds = Union(bounds, objects[i]->getBounds());
	//1个和2个物体时和BVH一样，只有一个或者一边一个
	if (objects.size() == 1) {
		// Create leaf _BVHBuildNode_
		node->bounds = objects[0]->getBounds();
		node->object = objects[0];
		node->left = nullptr;
		node->right = nullptr;
		return node;
	}
	else if (objects.size() == 2) {
		node->left = recursiveSAHBuild(std::vector{ objects[0] });
		node->right = recursiveSAHBuild(std::vector{ objects[1] });

		node->bounds = Union(node->left->bounds, node->right->bounds);
		return node;
	}
	else {
		Bounds3 centroidBounds;
		for (int i = 0; i < objects.size(); ++i)
			centroidBounds =
			Union(centroidBounds, objects[i]->getBounds().Centroid());
		float S = centroidBounds.SurfaceArea(); //注意S是所有节点包围盒的表面积
		//BVH:沿着包围盒在xyz上最大方向切分(沿着最长轴) 0 1 2分别对应x y z 方向上最大；xyz方向分别按质心排序，排序时间复杂度NlogN
		//SAH:在三个维度中计算使得cost最小的维度dim
		const int backet = 5; // 最好<32 博客看到的
		int bestDim = 0; //最好的维度
		int bestPartition = 0; //最好的划分点
		float minCost = std::numeric_limits<float>::infinity();
		for (int i = 0; i < 3; i++) {
			//i表示维度，按照维度方向给物体质心排序(只是物体的索引顺序变了、物体本身不会变)
			switch (i) {
			case 0:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().x <
						f2->getBounds().Centroid().x;
				});
				break;
			case 1:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().y <
						f2->getBounds().Centroid().y;
				});
				break;
			case 2:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().z <
						f2->getBounds().Centroid().z;
				});
				break;
			}

			for (int j = 0; j < backet; j++) {
				auto beginning = objects.begin();
				auto middling = objects.begin() + objects.size() * j / backet;
				auto ending = objects.end();
				auto leftshapes = std::vector<Object*>(beginning, middling);
				auto rightshapes = std::vector<Object*>(middling, ending);
				assert(objects.size() == (leftshapes.size() + rightshapes.size()));
				//分别求左右包围盒的表面积
				Bounds3 leftBounds, rightBounds;
				for (int k = 0; k < leftshapes.size(); k++) {
					leftBounds = Union(leftBounds, objects[k]->getBounds());
				}
				for (int k = 0; k < rightshapes.size(); k++) {
					rightBounds = Union(rightBounds, objects[k]->getBounds());
				}
				float leftS = leftBounds.SurfaceArea();
				float rightS = rightBounds.SurfaceArea();
				float cost = 0.125 + leftS / S * leftshapes.size() + rightS / S * rightshapes.size(); //0.125是遍历代价经验值 后面的是期望：概率*值
				if (cost < minCost) {
					minCost = cost;
					bestDim = i;
					bestPartition = j;
				}
			}
		}
		//现在知道minCost时的bestDim和bestPartition
		switch (bestDim) {
		case 0:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().x <
					f2->getBounds().Centroid().x;
			});
			break;
		case 1:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().y <
					f2->getBounds().Centroid().y;
			});
			break;
		case 2:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().z <
					f2->getBounds().Centroid().z;
			});
			break;
		}
		auto beginning = objects.begin();
		auto middling = objects.begin() + objects.size() * bestPartition / backet;
		auto ending = objects.end();

		auto leftshapes = std::vector<Object*>(beginning, middling);
		auto rightshapes = std::vector<Object*>(middling, ending);

		assert(objects.size() == (leftshapes.size() + rightshapes.size()));

		node->left = recursiveBuild(leftshapes);
		node->right = recursiveBuild(rightshapes);

		node->bounds = Union(node->left->bounds, node->right->bounds);
	}
	return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

//建立 BVH 之后，我们可以用它加速求交过程。该过程递归进行，你将在其中调用你实现的 Bounds3::IntersectP
//返回的intersection要有相交的材质、物体等
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
	// root和ray有交点吗？-->没有：跳出；有：递归看root左孩子和root右孩子和ray有交点吗
	Intersection isect;
	//base case
	if (node == nullptr) {
		return isect;
	}
	// 前序位置
	// 用来判断光线是否反向
	std::array<int, 3> dirIsNeg;
	for (int i = 0; i < 3; i++) {
		dirIsNeg[i] = int(ray.direction[i] >= 0);
	}
	// 如果ray和当前node的bounding box 没有交点，跳出
	if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
		return isect;
	}
	// 如果是叶子节点，找叶子节点里面所有的object和ray的交点，返回最近的（这里BVHBuildNode类中是Object类，也就是一个node只有一个object）
	if (node->left == nullptr && node->right == nullptr) {
		isect = node->object->getIntersection(ray);
		//isect.happened = true; //？后加 需要吗--》
		//不需要！上一个if决定了这个node一定和包围盒有交点；如果node是叶子节点、上一行：和包围盒里的object求交点
		//（物体类如sphere、triangle里面把交点.happened设为true了）
		return isect;
	}
	//否则，左右子结点递归求bounding box 和 ray 交点，返回最近的(因为whitted style只找最近的交点、再根据表面材质选择反射折射等)
	Intersection hit1 = BVHAccel::getIntersection(node->left, ray);
	Intersection hit2 = BVHAccel::getIntersection(node->right, ray);

	return hit1.distance < hit2.distance ? hit1 : hit2;
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}