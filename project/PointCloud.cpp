#include "PointCloud.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

PointCloud::PointCloud() {
	size = 0;
}

PointCloud::PointCloud(std::vector<Point*> p) {
	size = p.size();
	points = p;
}

PointCloud::~PointCloud() {
	unsigned int i = 0;
	while (i != size)
		delete points[i++];
}

unsigned int PointCloud::getSize() const {
	return points.size();
}

void PointCloud::addPoint(Point* p) {
	points.push_back(p);
	size++;
}

std::vector<Point*> PointCloud::getPoints() const {
	return points;
}

void PointCloud::clear() {
	points.clear();
}

Point* PointCloud::getCentroid() const
{
	Point* centroid = new Point(points[0]->getLabel(),points[0]->getX(),points[0]->getY(),points[0]->getZ());
	for(int i=1;i<size;++i)
	{
		centroid->getCoordinates()[0] += points[i]->getCoordinates()[0];
		centroid->getCoordinates()[1] += points[i]->getCoordinates()[1];
		centroid->getCoordinates()[2] += points[i]->getCoordinates()[2];
	}
	centroid->getCoordinates()[0] /= size;
	centroid->getCoordinates()[1] /= size;
	centroid->getCoordinates()[2] /= size;
	return centroid;
}
