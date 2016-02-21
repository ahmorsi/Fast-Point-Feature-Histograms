#include <gtest/gtest.h>
#include <boost/random.hpp>
#include <Eigen/Eigenvalues>
#include "../project/Point.h"
#include "../project/PointCloud.h"
#include "../project/KdTree.h"
#include "../project/Features.h"
#include "../project/SPFH.h"
#include "../project/FPFH.h"

void randomPoints(PointCloud&, uint32_t, uint32_t);

bool is_angle_in_range(double value) {
	if (value < -M_PI || value > M_PI) {
		return false;
	}
	return true;
}

TEST(HistogramTest, Features) {
	uint32_t N = 1000;

	boost::mt11213b mtwister(1234);
	boost::uniform_int<> uni_dist(0, N - 1);

	PointCloud points;
	randomPoints(points, N, 1234);

	KdTreeNode kd;

	kd.buildTree(points.getPoints(), 0);

	float radius = 2.0;

	for (uint32_t i = 0; i < 10; ++i) {
		std::vector<Point*> neighborsKdTree;

		Point* query = points.getPoints()[uni_dist(mtwister)];
		kd.fixedRadiusSearch(query, radius, neighborsKdTree);
		query->computeNormal(neighborsKdTree);
	}

	float radius2 = 1.0;

	for (uint32_t r = 0; r < 4; ++r) {
		for (uint32_t i = 0; i < 10; ++i) {
			std::vector<Point*> neighborsKdTree;
			Point* query = points.getPoints()[uni_dist(mtwister)];
			kd.fixedRadiusSearch(query, radius2, neighborsKdTree);
			for (unsigned int j = 0; j < neighborsKdTree.size(); j++) {
				Features f(query, neighborsKdTree[j]);
				ASSERT_TRUE(is_angle_in_range(f.getAlpha()))<< " feature alpha is not in range [-Pi; Pi].";
				ASSERT_TRUE(
						is_angle_in_range(f.getPhi()))<< " feature phi is not in range [-Pi; Pi].";
				ASSERT_TRUE(
						is_angle_in_range(f.getTheta()))<< " feature theta is not in range [-Pi; Pi].";
			}
		}
	}
}

void check_spfh(SPFH* s, int size) {

	double* alpha = s->getHistogramAlpha();
	double* theta = s->getHistogramTheta();
	double* phi = s->getHistogramPhi();

	int countAlpha = 0;
	int countTheta = 0;
	int countPhi = 0;

	for (uint32_t i = 0; i < s->getHistogramSize(); i++) {
		ASSERT_FALSE(isnan(alpha[i]))<< " there are NAN entries in the alpha spfh";
		ASSERT_FALSE(isnan(phi[i])) << " there are NAN entries in the phi spfh";
		ASSERT_FALSE(isnan(theta[i])) << " there are NAN entries in the theta spfh";

		if (alpha[i] != 0) {
			countAlpha += alpha[i];
		}
		if (theta[i] != 0) {
			countTheta += theta[i];
		}
		if (phi[i] != 0) {
			countPhi += phi[i];
		}
	}

	ASSERT_EQ(size, countAlpha)<< " there are entries missing in the alpha histogram";
	ASSERT_EQ(size, countTheta)<< " there are entries missing in the theta histogram";
	ASSERT_EQ(size, countPhi)<< " there are entries missing in the phi histogram";
}

void check_fpfh(Point* query, std::vector<Point*> neighbors) {

	SPFH* s = query->getSimplePointFeatureHistogram();
	FPFH* f = query->getFastPointFeatureHistogram();

	double* simpleAlpha = s->getHistogramAlpha();
	double* simplePhi = s->getHistogramPhi();
	double* simpleTheta = s->getHistogramTheta();

	double* fastAlpha = f->getHistogramAlpha();
	double* fastPhi = f->getHistogramPhi();
	double* fastTheta = f->getHistogramTheta();

	ASSERT_EQ(s->getHistogramSize(), f->getHistogramSize())<< " the histograms have different sizes";

	int size = s->getHistogramSize();

	for (uint32_t i = 0; i < size; i++) {
		ASSERT_FALSE(isnan(fastAlpha[i]))<< " there are NAN entries in the alpha fpfh";
		ASSERT_FALSE(isnan(fastPhi[i])) << " there are NAN entries in the phi fpfh";
		ASSERT_FALSE(isnan(fastTheta[i])) << " there are NAN entries in the theta fpfh";

		if (simpleAlpha[i] != 0) {
			ASSERT_NE(0, fastAlpha[i])<< " fast alpha histogram can't have entry 0 if simple histogram is not 0";
		}
		if(simplePhi[i] != 0) {
			ASSERT_NE(0, fastPhi[i]) << " fast ph histogram can't have entry 0 if simple histogram is not 0";
		}
		if(simpleTheta[i] != 0) {
			ASSERT_NE(0, fastTheta[i]) << " fast theta histogram can't have entry 0 if simple histogram is not 0";
		}
	}

	for (uint32_t j = 0; j < neighbors.size(); j++) {
		double* simpleAlpha_n =
				neighbors[j]->getSimplePointFeatureHistogram()->getHistogramAlpha();
		double* simplePhi_n =
				neighbors[j]->getSimplePointFeatureHistogram()->getHistogramPhi();
		double* simpleTheta_n =
				neighbors[j]->getSimplePointFeatureHistogram()->getHistogramTheta();
		for (uint32_t i = 0; i < size; i++) {
			if (simpleAlpha_n[i] != 0) {
				ASSERT_NE(0, fastAlpha[i])<< " fast alpha histogram can't have entry 0 if simple histogram is not 0";
			}
			if(simplePhi_n[i] != 0) {
				ASSERT_NE(0, fastPhi[i]) << " fast ph histogram can't have entry 0 if simple histogram is not 0";
			}
			if(simpleTheta_n[i] != 0) {
				ASSERT_NE(0, fastTheta[i]) << " fast theta histogram can't have entry 0 if simple histogram is not 0";
			}
		}
	}

}

TEST(HistogramTest, SPFH) {
	uint32_t N = 1000;

	double binsize = 0.1;

	boost::mt11213b mtwister(1234);
	boost::uniform_int<> uni_dist(0, N - 1);

	PointCloud points;
	randomPoints(points, N, 1234);

	KdTreeNode kd;

	kd.buildTree(points.getPoints(), 0);

	float radius = 3.0;

	for (uint32_t i = 0; i < N; ++i) {
		std::vector<Point*> neighborsKdTree;

		Point* query = points.getPoints()[i];
		kd.fixedRadiusSearch(query, radius, neighborsKdTree);
		query->computeNormal(neighborsKdTree);
	}

	float radius2 = 3.0;

	for (uint32_t i = 0; i < 10; ++i) {
		std::vector<Point*> neighborsKdTree;
		Point* query = points.getPoints()[uni_dist(mtwister)];
		kd.fixedRadiusSearch(query, radius2, neighborsKdTree);

		bool pointContained = false;

		SPFH* s = new SPFH(-M_PI, M_PI, binsize);
		for (unsigned int j = 0; j < neighborsKdTree.size(); j++) {
			if (neighborsKdTree[j] == query) {
				pointContained = true;
				continue;
			}
			Features f(query, neighborsKdTree[j]);
			s->addToHistogram(&f);
		}
		if(pointContained == true)
			check_spfh(s, neighborsKdTree.size()-1);
		else
			check_spfh(s, neighborsKdTree.size());
		delete (s);
	}
}

TEST(HistogramTest, FPFH) {
	uint32_t N = 1000;

	double binsize = 0.1;

	boost::mt11213b mtwister(1234);
	boost::uniform_int<> uni_dist(0, N - 1);

	PointCloud points;
	randomPoints(points, N, 1234);

	KdTreeNode kd;

	kd.buildTree(points.getPoints(), 0);

	float radius = 3.0;

	for (uint32_t i = 0; i < N; ++i) {
		std::vector<Point*> neighborsKdTree;

		Point* query = points.getPoints()[i];
		kd.fixedRadiusSearch(query, radius, neighborsKdTree);
		query->computeNormal(neighborsKdTree);
	}

	float radius2 = 2.0;

	for (uint32_t i = 0; i < N; ++i) {
		std::vector<Point*> neighborsKdTree;
		kd.fixedRadiusSearch(points.getPoints()[i], radius2, neighborsKdTree);

		SPFH* s = new SPFH(-M_PI, M_PI, binsize);
		for (unsigned int j = 0; j < neighborsKdTree.size(); j++) {
			if (neighborsKdTree[j] == points.getPoints()[i]) {
				continue;
			}
			Features f(points.getPoints()[i], neighborsKdTree[j]);
			s->addToHistogram(&f);
		}
		points.getPoints()[i]->setSimplePointFeatureHistogram(s);
	}

	std::vector<Point*> pointList;
	for (uint32_t i = 0; i < 10; ++i) {
		pointList.push_back(points.getPoints()[uni_dist(mtwister)]);
	}

	for (uint32_t i = 0; i < 10; ++i) {
		std::vector<Point*> neighborsKdTree;
		kd.fixedRadiusSearch(pointList[i], radius2, neighborsKdTree);

		FPFH* f = new FPFH(-M_PI, M_PI, binsize);
		f->addToHistogram(pointList[i]->getSimplePointFeatureHistogram(), 1, 1);
		for (unsigned int j = 0; j < neighborsKdTree.size(); j++) {
			if (neighborsKdTree[j] == pointList[i]) {
				continue;
			}

			f->addToHistogram(
					neighborsKdTree[j]->getSimplePointFeatureHistogram(),
					neighborsKdTree.size(),
					pointList[i]->euclideanDistance(neighborsKdTree[j]));
		}
		pointList[i]->setFastPointFeatureHistogram(f);
		check_fpfh(pointList[i], neighborsKdTree);
	}
}
