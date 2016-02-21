#include "Features.h"
#include <cmath>

Features::Features(Point* p1, Point* p2): alpha(NAN), theta(NAN), phi(NAN){
	//determine source and target point
	Point* ps;
	Point* pt;

	//TODO: Determine source and target point
	double lhs = p1->getNormal().dot(p2->getCoordinates()-p1->getCoordinates());
	double rhs = p2->getNormal().dot(p1->getCoordinates()-p2->getCoordinates());
	if(acos(lhs) <= acos(rhs))
	{
		ps = p1;
		pt=p2;
	}
	else
	{
		ps=p2;
		pt=p1;
	}
	//compute alpha, phi, theta
	computeFeatures(ps, pt);
}

Features::~Features() {
}

void Features::computeFeatures(Point* ps, Point* pt) {

	//TODO: Compute alpha, phi and theta
	Eigen::Vector3f u,v,w,unitDist;
	u = ps->getNormal();
	unitDist = pt->getCoordinates()-ps->getCoordinates();
	unitDist.normalize();

	v= unitDist.cross(u);
	w= u.cross(v);

	alpha = v.dot(pt->getNormal());
	phi = u.dot(unitDist);
	theta = atan2(u.dot(pt->getNormal()),w.dot(pt->getNormal()));
}
