#include <gtest/gtest.h>
#include <boost/random.hpp>
#include "../project/KdTree.h"
#include "../project/PointCloud.h"

// simple bruteforce search.
void radiusNeighbors(Point* query, float radius, PointCloud& cloud,
		std::vector<Point*>& resultPoints) {

	std::vector<Point*> points = cloud.getPoints();
	resultPoints.clear();

	for (uint32_t i = 0; i < points.size(); ++i) {
		if (query->euclideanDistance(points[i]) < radius) {
			resultPoints.push_back(points[i]);
		}
	}
}

void randomPoints(PointCloud& pts, uint32_t N, uint32_t seed = 0) {
	boost::mt11213b mtwister(seed);
	boost::uniform_01<> gen;
	pts.clear();
// generate N random points in [-5.0,5.0] x [-5.0,5.0] x [-5.0,5.0]...
	for (uint32_t i = 0; i < N; ++i) {
		float x = 10.0f * gen(mtwister) - 5.0f;
		float y = 10.0f * gen(mtwister) - 5.0f;
		float z = 10.0f * gen(mtwister) - 5.0f;

		pts.addPoint(new Point(0, x, y, z));
	}
}

bool foundPoint(const KdTreeNode* node, Point* p) {
	if (node == NULL)
		return false;
	if (node->location == p) {
		return true;
	} else {
		bool ret = false;
		ret = foundPoint(node->right_node, p);
		if (ret == false) {
			ret = foundPoint(node->left_node, p);
		}
		return ret;
	}
}

void pointContained(const KdTreeNode* node, const Point* location) {
	if (location == NULL) {
		return;
	}
	//check if each child nodes coordinates are part of its parents successor list
	const std::vector<Point*>& successors = node->getSuccessors();
	bool contained = false;
	for (uint32_t t = 0; t < successors.size(); t++) {
		if (successors[t] == location) {
			contained = true;
			break;
		}
	}
	ASSERT_EQ(true, contained)<< " point with coordinates " << location << " is not part of the successor list of its parent";
}

void checkSuccessorList(const std::vector<Point*> successorParent,
		const std::vector<Point*> successorChild, Point* location) {
	if (location == NULL) {
		return;
	}
	//check if the successor list of the child is a real subset of the successor list of the parent
	for (uint32_t t = 0; t < successorChild.size(); t++) {
		bool contained = false;
		for (uint32_t t2 = 0; t2 < successorParent.size(); t2++) {
			if (successorParent[t2] == successorChild[t]) {
				contained = true;
				break;
			}
		}
		ASSERT_EQ(true, contained)<< " successor list of point with coordinates " << *location << " is not a real subset of parents successor list";
	}
}

void checkSuccessorListSorted(const std::vector<Point*> successors, int depth,
		Point* location) {
	int axis = depth % 3;
	//check if successor lists are sorted correctly
	if (successors.size() > 0) {
		for (uint32_t t = 0; t < successors.size() - 1; t++) {
			ASSERT_LE((float)successors[t]->getCoordinates()[axis], (float)successors[t+1]->getCoordinates()[axis])<< " successor list of point with coordinates " << *location << " is not sorted correctly";
		}
	}
}

void checkContent(const KdTreeNode* node) {

	if (node->location == NULL)
		return;

	if (node->right_node != NULL) {
		Point* p = node->right_node->location;
		pointContained(node, p);
		checkSuccessorList(node->getSuccessors(),
				node->right_node->getSuccessors(), p);
		checkSuccessorListSorted(node->right_node->getSuccessors(),
				node->right_node->getDepth(), p);
		checkContent(node->right_node);
	}

	if (node->left_node != NULL) {
		Point* p = node->left_node->location;
		pointContained(node, p);
		checkSuccessorList(node->getSuccessors(),
				node->left_node->getSuccessors(), p);
		checkSuccessorListSorted(node->left_node->getSuccessors(),
				node->left_node->getDepth(), p);
		checkContent(node->left_node);
	}
}

TEST(KdTreeTest, Initialize) {
	uint32_t N = 1000;

	KdTreeNode kd;

	ASSERT_EQ(NULL, kd.location);

	PointCloud points;
	randomPoints(points, N, 1337);

	kd.buildTree(points.getPoints(), 0);

	const std::vector<Point*>& successors = kd.getSuccessors();

	// check first some pre-requisits.
	ASSERT_EQ(true, (kd.location != 0))<< " a point should be assigned to root";
	ASSERT_EQ(0, kd.getDepth())<<" root depth should be zero";
	ASSERT_EQ(N-1, successors.size())<< "successors should be of size " << N-1;

	std::vector<uint32_t> elementCount(N, 0);
	// check that each point was found.
	for (uint32_t t = 0; t < N; t++) {
		Point* p = points.getPoints()[t];
		if (foundPoint(&kd, p)) {
			elementCount[t]++;
			ASSERT_EQ(1, elementCount[t])<< "point "<< t << " found twice in successors.";
		}
	}

	// check that each index was found.
	for (uint32_t i = 0; i < N; ++i) {
		ASSERT_EQ(1, elementCount[i])<< "point with index " << i << " not found.";
	}

	// test if kd tree is ordered correctly
	checkContent(&kd);
}

void similarVectors(std::vector<Point*>& vec1, std::vector<Point*>& vec2) {
	ASSERT_EQ(vec1.size(), vec2.size())<< "expected size = " << vec1.size() << ", but got size = " << vec2.size() << "\n";

	for (uint32_t i = 0; i < vec1.size(); ++i)
	{
		bool found = false;
		for (uint32_t j = 0; j < vec2.size(); ++j)
		{
			if (vec1[i] == vec2[j])
			{
				found = true;
				break;
			}
		}

		ASSERT_EQ(true, found) << i << "-th element (" << *vec1[i] << ") not found." << "ņ";
	}
}

TEST(KdTreeTest, RadiusNeighbors) {
	uint32_t N = 1000;

	boost::mt11213b mtwister(1234);
	boost::uniform_int<> uni_dist(0, N - 1);

	PointCloud points;
	randomPoints(points, N, 1234);

	KdTreeNode kd;

	kd.buildTree(points.getPoints(), 0);

	float radii[4] = { 0.5, 1.0, 2.0, 5.0 };

	for (uint32_t r = 0; r < 4; ++r) {
		for (uint32_t i = 0; i < 10; ++i) {
			std::vector<Point*> neighborsBruteforce;
			std::vector<Point*> neighborsKdTree;

			Point* query = points.getPoints()[uni_dist(mtwister)];

			radiusNeighbors(query, radii[r], points, neighborsBruteforce);
			kd.fixedRadiusSearch(query, radii[r], neighborsKdTree);

//			kd.printTree();

//			std::cout << "query point " << *query << std::endl;
//			std::cout << "radius: " << radii[r] << std::endl;
//			std::cout << "kd tree neighbors: " << std::endl;
//			for (int f = 0; f < neighborsKdTree.size(); f++) {
//				std::cout << *neighborsKdTree[f] << std::endl;
//			}
//
//			std::cout << "bruteforce neighbors: " << std::endl;
//			for (int f = 0; f < neighborsBruteforce.size(); f++) {
//				std::cout << *neighborsBruteforce[f] << std::endl;
//			}

			similarVectors(neighborsBruteforce, neighborsKdTree);
		}
	}
}

