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

// SAH ���������ʽ�㷨
BVHBuildNode* BVHAccel::recursiveSAHBuild(std::vector<Object*> objects)
{
	BVHBuildNode* node = new BVHBuildNode();

	// Compute bounds of all primitives in BVH node
	Bounds3 bounds;
	//��ȡ�������������������Χ��
	for (int i = 0; i < objects.size(); ++i)
		//Bounds3::Union �����������ǽ�������Χ�в��ɸ���İ�Χ�С������һ���������е�ÿ������ʵ�������Լ��İ�Χ�С�
		bounds = Union(bounds, objects[i]->getBounds());
	//1����2������ʱ��BVHһ����ֻ��һ������һ��һ��
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
		float S = centroidBounds.SurfaceArea(); //ע��S�����нڵ��Χ�еı����
		//BVH:���Ű�Χ����xyz��������з�(�������) 0 1 2�ֱ��Ӧx y z ���������xyz����ֱ�������������ʱ�临�Ӷ�NlogN
		//SAH:������ά���м���ʹ��cost��С��ά��dim
		const int backet = 5; // ���<32 ���Ϳ�����
		int bestDim = 0; //��õ�ά��
		int bestPartition = 0; //��õĻ��ֵ�
		float minCost = std::numeric_limits<float>::infinity();
		for (int i = 0; i < 3; i++) {
			//i��ʾά�ȣ�����ά�ȷ����������������(ֻ�����������˳����ˡ����屾�����)
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
				//�ֱ������Ұ�Χ�еı����
				Bounds3 leftBounds, rightBounds;
				for (int k = 0; k < leftshapes.size(); k++) {
					leftBounds = Union(leftBounds, objects[k]->getBounds());
				}
				for (int k = 0; k < rightshapes.size(); k++) {
					rightBounds = Union(rightBounds, objects[k]->getBounds());
				}
				float leftS = leftBounds.SurfaceArea();
				float rightS = rightBounds.SurfaceArea();
				float cost = 0.125 + leftS / S * leftshapes.size() + rightS / S * rightshapes.size(); //0.125�Ǳ������۾���ֵ �����������������*ֵ
				if (cost < minCost) {
					minCost = cost;
					bestDim = i;
					bestPartition = j;
				}
			}
		}
		//����֪��minCostʱ��bestDim��bestPartition
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

//���� BVH ֮�����ǿ������������󽻹��̡��ù��̵ݹ���У��㽫�����е�����ʵ�ֵ� Bounds3::IntersectP
//���ص�intersectionҪ���ཻ�Ĳ��ʡ������
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
	// root��ray�н�����-->û�У��������У��ݹ鿴root���Ӻ�root�Һ��Ӻ�ray�н�����
	Intersection isect;
	//base case
	if (node == nullptr) {
		return isect;
	}
	// ǰ��λ��
	// �����жϹ����Ƿ���
	std::array<int, 3> dirIsNeg;
	for (int i = 0; i < 3; i++) {
		dirIsNeg[i] = int(ray.direction[i] >= 0);
	}
	// ���ray�͵�ǰnode��bounding box û�н��㣬����
	if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
		return isect;
	}
	// �����Ҷ�ӽڵ㣬��Ҷ�ӽڵ��������е�object��ray�Ľ��㣬��������ģ�����BVHBuildNode������Object�࣬Ҳ����һ��nodeֻ��һ��object��
	if (node->left == nullptr && node->right == nullptr) {
		isect = node->object->getIntersection(ray);
		//isect.happened = true; //����� ��Ҫ��--��
		//����Ҫ����һ��if���������nodeһ���Ͱ�Χ���н��㣻���node��Ҷ�ӽڵ㡢��һ�У��Ͱ�Χ�����object�󽻵�
		//����������sphere��triangle����ѽ���.happened��Ϊtrue�ˣ�
		return isect;
	}
	//���������ӽ��ݹ���bounding box �� ray ���㣬���������(��Ϊwhitted styleֻ������Ľ��㡢�ٸ��ݱ������ѡ���������)
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