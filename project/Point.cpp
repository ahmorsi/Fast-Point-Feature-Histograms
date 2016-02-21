#include "Point.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include<complex>
#include "Utility.h"

Point::Point() :
		simpleHistogram(), fastHistogram(), eigenValue(0), label(0) {
	for (int i = 0; i < 3; i++) {
		coordinates[i] = 0;
	}
	normal = Eigen::Vector3f::Zero();
}

Point::Point(int label, double* coords) :
		simpleHistogram(), fastHistogram(), eigenValue(0), label(label) {
	for (int i = 0; i < 3; i++) {
		coordinates[i] = coords[i];
	}
	normal = Eigen::Vector3f::Zero();
}

Point::Point(int label, double x, double y, double z) :
		simpleHistogram(), fastHistogram(), eigenValue(0), label(label) {
	coordinates[0] = x;
	coordinates[1] = y;
	coordinates[2] = z;
	normal = Eigen::Vector3f::Zero();
}
Point::Point(Point& rhs):simpleHistogram(rhs.simpleHistogram), fastHistogram(rhs.fastHistogram), eigenValue(rhs.eigenValue), label(rhs.label)
{
	coordinates = rhs.coordinates;
	normal = rhs.normal;
}
Point::~Point() {
	delete simpleHistogram;
	delete fastHistogram;
}

void Point::computeNormal(std::vector<Point*> neighbors) {

	Point* cog = computeCenterOfGravity(neighbors);
	Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
	computeCovarianceMatrix(neighbors, cog, covariance);

	Eigen::EigenSolver<Eigen::Matrix3f> eigensolver (covariance);

	int minEigenValueIdx =0;

	std::complex<double> lamda[3];
	std::complex<double> minLamda;
	minLamda = lamda[0]= eigensolver.eigenvalues()[0];
	lamda[1] = eigensolver.eigenvalues()[1];
	lamda[2] = eigensolver.eigenvalues()[2];
	for(int i=1;i<3;++i)
		if(minLamda.real() > lamda[i].real())
		{
			minLamda = lamda[i];
			minEigenValueIdx = i;
		}
	eigenValue = lamda[minEigenValueIdx].real();
	for(int i=0;i<this->normal.size();++i)
	{
		std::complex<double> complexValue = eigensolver.eigenvectors().col(minEigenValueIdx)[i];
		this->normal[i] = complexValue.real();
	}
	//make sure the normals are oriented consistently
	Eigen::Vector3f viewPoint = Eigen::Vector3f::Zero();
	float angle = this->normal.transpose() * (viewPoint - this->coordinates);
	if(angle < 0)
		this->normal = this->normal * -1;
	delete cog;
}

double Point::euclideanDistance(Point* p) {
	double dist = 0;

	Eigen::Vector3f tmp = p->getCoordinates() - coordinates;

	dist = pow((float) tmp[0], 2) + pow((float) tmp[1], 2)
			+ pow((float) tmp[2], 2);

	return sqrt(dist);
}

Point& Point::operator =(const Point& src) {

	for (int i = 0; i < 3; i++) {
		coordinates[i] = src.getCoordinates()[i];
	}

	normal = src.getNormal();
	label = src.getLabel();

	fastHistogram = src.getFastPointFeatureHistogram();
	simpleHistogram = src.getSimplePointFeatureHistogram();

	return *this;
}

bool Point::operator ==(const Point& b) {
	Eigen::Vector3f o_coordinates = b.getCoordinates();
	if (coordinates[0] == o_coordinates[0] && coordinates[1] == o_coordinates[1]
			&& coordinates[2] == o_coordinates[2]) {
		return true;
	}
	return false;
}

bool Point::operator !=(const Point& b) {
	return !(*this == b);
}
