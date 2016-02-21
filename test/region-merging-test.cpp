#include <gtest/gtest.h>
#include <boost/random.hpp>
#include "../project/Point.h"
#include "../project/PointCloud.h"
#include "../project/KdTree.h"
#include "../project/RegionMerging.h"
#include <vector>

void randomPointsWithLabels(PointCloud& pts, uint32_t N, uint32_t seed = 0) {
	boost::mt11213b mtwister(seed);
	boost::uniform_01<> gen;
	pts.clear();
// generate N random points in [-5.0,5.0] x [-5.0,5.0] x [-5.0,5.0]...
	for (uint32_t i = 0; i < N; ++i) {
		float x = 10.0f * gen(mtwister) - 5.0f;
		float y = 10.0f * gen(mtwister) - 5.0f;
		float z = 10.0f * gen(mtwister) - 5.0f;

		float sw = gen(mtwister);

		if (sw > 0.5)
			pts.addPoint(new Point(1, x, y, z));
		else
			pts.addPoint(new Point(-1, x, y, z));
	}
}

TEST(RegionMergingTest, RegionMerging) {
	uint32_t N = 1000;
	float rN = 2.0;

	boost::mt11213b mtwister(1234);
	boost::uniform_int<> uni_dist(0, N - 1);

	PointCloud points;
	randomPointsWithLabels(points, N, 1234);

	KdTreeNode kd;

	kd.buildTree(points.getPoints(), 0);

	for (unsigned int i = 0; i < points.getSize(); i++) {

		std::vector<Point*> neighbors;
		kd.fixedRadiusSearch(points.getPoints()[i], rN, neighbors);
		points.getPoints()[i]->computeNormal(neighbors);
	}

	std::vector<PointCloud*> clusters;
	std::vector<Point*> centroids;

	RegionMerging merging(rN);
	merging.performRegionMerging(points, kd, clusters, centroids);

	ASSERT_LE(clusters.size(), N)<< "Number of clusters has to be less or equal to number of points ";
	ASSERT_GT(clusters.size(), 0)<< "There has to be at least one cluster ";

	int sum = 0;
	for (int i = 0; i < clusters.size(); i++) {
		sum += clusters[i]->getSize();
	}

	ASSERT_EQ(N, sum)<< "Every point has to be included in exactly one cluster ";

	int indezes[N];

	for (int i = 0; i < N; i++) {
		indezes[i] = 0;
		Point* p = points.getPoints()[i];
		bool br = false;
		for (int j = 0; j < clusters.size(); j++) {
			for (int k = 0; k < clusters[j]->getPoints().size(); k++) {
				if (p->getCoordinates() == clusters[j]->getPoints()[k]->getCoordinates()) {
					indezes[i]++;
					ASSERT_EQ(indezes[i], 1) << "Point number " << i << " with coordinates " << *p << " is included more than once ";
					br = true;
				}
				if (br == true) {
					break;
				}
			}
			if (br == true) {
				break;
			}
		}
		ASSERT_EQ(1, indezes[i]) << "Point number " << i << " with coordinates " << *p << " is not included in the point clusters ";
	}

	for(int i = 0; i<clusters.size(); i++) {
		delete(clusters[i]);
		delete(centroids[i]);
	}
}

