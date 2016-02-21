#ifndef POINT_H_
#define POINT_H_

#include <vector>
#include <Eigen/Eigenvalues>
#include <iostream>
#include "SPFH.h"
#include "FPFH.h"

class SPFH;
class FPFH;

/** \brief Implementation of a labeled point
 *
 *  Stores a labeled 3d-point, i.e. a point with coordinates x, y, z and a label l associating it
 *  with a label class.
 *
 *  \author mack
 */
enum AXIS {X=0,Y=1,Z=2};
class Point {
	Eigen::Vector3f coordinates;
	Eigen::Vector3f normal;
	int label;
	double eigenValue;
	SPFH* simpleHistogram;
	FPFH* fastHistogram;

public:
	Point();
	Point(int, double*);
	Point(int, double, double, double);
	Point(Point& rhs);
	virtual ~Point();

	float getX() const {
		return coordinates[0];
	}
	float getY() const {
		return coordinates[1];
	}
	float getZ() const {
		return coordinates[2];
	}
	float getAxisVal(int axis) const
	{
		return coordinates[axis];
	}
	Eigen::Vector3f getNormal() const {
		return normal;
	}

	double getEigenValue() const {
		return eigenValue;
	}

	Eigen::Vector3f getCoordinates() const {
		return coordinates;
	}

	int getLabel() const {
		return label;
	}

	void setLabel(int l) {
		label = l;
	}

	SPFH* getSimplePointFeatureHistogram() const {
		return simpleHistogram;
	}

	void setSimplePointFeatureHistogram(SPFH* simpleHistogram) {
		this->simpleHistogram = simpleHistogram;
	}

	FPFH* getFastPointFeatureHistogram() const {
		return fastHistogram;
	}

	void setFastPointFeatureHistogram(FPFH* fastHistogram) {
		this->fastHistogram = fastHistogram;
	}

	//compute normal for the point from a given set of neighbors
	void computeNormal(std::vector<Point*>);

	//compute euclidean distance between this and the specified point
	double euclideanDistance(Point*);

	Point& operator =(const Point&);

	bool operator ==(const Point&);

	bool operator !=(const Point&);

	friend std::ostream& operator<<(std::ostream& out, const Point& p) {
		out.width(4);
		out.precision(3);
		out << p.coordinates[0] << ", " << p.coordinates[1] << ", "
				<< p.coordinates[2];
		return out;
	}
};

#endif /* POINT_H_ */
