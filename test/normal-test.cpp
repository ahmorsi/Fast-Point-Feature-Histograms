#include <gtest/gtest.h>
#include <boost/random.hpp>
#include <Eigen/Eigenvalues>
#include "../project/Point.h"
#include "../project/PointCloud.h"
#include "../project/KdTree.h"

void randomPoints(PointCloud&, uint32_t, uint32_t);
Point* computeCenterOfGravity(std::vector<Point*>);
void computeCovarianceMatrix(std::vector<Point*>, Point*, Eigen::Matrix3f&);

bool similarVectors(Eigen::Vector3f v1, Eigen::Vector3f v2,
		float allowed_diff) {
	float difference = 0;
	for (uint32_t i = 0; i < 3; i++) {
		double dff = v1[i] - v2[i];
		difference += fabs(dff);
	}
	return difference <= allowed_diff;
}

void testNormal(Point* query, std::vector<Point*> neighbors) {
	Eigen::Vector3f normal = query->getNormal();

	Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
	Point* cog = computeCenterOfGravity(neighbors);
	computeCovarianceMatrix(neighbors, cog, cov);
	delete (cog);

	Eigen::Vector3f normalVec;
	normalVec << normal[0], normal[1], normal[2];

	Eigen::Vector3f mult = cov * normalVec;

	Eigen::Vector3f res = query->getEigenValue() * normalVec;

	ASSERT_EQ(true, similarVectors(res, mult, 10e-05))<< " normal does not fulfill the condition \\lambda * n = C * n";

	Eigen::Vector3f viewPoint = Eigen::Vector3f::Zero();
	Eigen::Vector3f point;
	point << query->getCoordinates()[0], query->getCoordinates()[1], query->getCoordinates()[2];
	float angle = normalVec.transpose() * (viewPoint - point);
	ASSERT_GT(angle, 0)<< " normals are inconsistently oriented";
}

void testCovariance(Point* query, std::vector<Point*> neighbors) {
	Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
	Point* cog = computeCenterOfGravity(neighbors);
	computeCovarianceMatrix(neighbors, cog, cov);

	ASSERT_EQ(cov, cov.transpose())<< " covariance matrix is not symmetric";

	delete (cog);

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
	es.compute(cov);
	Eigen::Vector3f eigenValues = es.eigenvalues();

	for (uint32_t i = 0; i < eigenValues.size(); i++) {
		float eigenValue = eigenValues[0];

		ASSERT_GE(eigenValue, -10e-05)<< " covariance matrix is not positive semi-definite";
	}
}

TEST(NormalTest, Covariance) {
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
			std::vector<Point*> neighborsKdTree;

			Point* query = points.getPoints()[uni_dist(mtwister)];

			kd.fixedRadiusSearch(query, radii[r], neighborsKdTree);

			testCovariance(query, neighborsKdTree);
		}
	}
}

TEST(NormalTest, Normals) {
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
			std::vector<Point*> neighborsKdTree;

			Point* query = points.getPoints()[uni_dist(mtwister)];

			kd.fixedRadiusSearch(query, radii[r], neighborsKdTree);

			query->computeNormal(neighborsKdTree);

			testNormal(query, neighborsKdTree);
		}
	}
}

