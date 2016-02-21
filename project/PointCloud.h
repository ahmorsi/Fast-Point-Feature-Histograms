#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include <vector>
#include "Point.h"

/** \brief Implementation of a point cloud
 *
 *  The point cloud stores a set of points and provides methods for sorting them
 *
 *  \author mack
 */

class PointCloud {
	std::vector<Point*> points;
	unsigned int size;

public:
	PointCloud();
	PointCloud(std::vector<Point*>);

	virtual ~PointCloud();

	std::vector<Point*> getPoints() const;

	unsigned int getSize() const;

	Point* getCentroid() const;
	//add point to point cloud
	void addPoint(Point*);

	void clear();

	friend std::ostream& operator<<(std::ostream& out, const PointCloud& p) {
		out.width(4);
		out.precision(3);
		for (unsigned int i = 0; i < p.getSize(); i++) {
			out << *p.getPoints()[i];
		}
		return out;
	}
};

#endif /* POINTCLOUD_H_ */
