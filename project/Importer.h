#ifndef IMPORTER_H_
#define IMPORTER_H_

#include <string>
#include <stdlib.h>
#include "PointCloud.h"

/** \brief Implementation of an importer of point clouds from a given file
 *
 *  The importer takes a path to a file as input and loads a point cloud out of the given file
 *
 *  \author mack
 */

class Importer {
	std::string filename;

public:
	Importer(std::string);
	virtual ~Importer();

	std::string getFilename();

	void readFile(PointCloud&);

	//read point cloud associated with prediction file
	int readFileWithPrediction(std::string, int, PointCloud&);
};

#endif /* IMPORTER_H_ */
