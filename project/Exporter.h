#ifndef EXPORTER_H_
#define EXPORTER_H_

#include <string>
#include <stdlib.h>
#include "PointCloud.h"

/** \brief Implementation of an exporter for the FPFHs from a point cloud to a feature file
 *
 *  The exporter takes a point cloud and exports the FPFHs of all points to a feature file
 *
 *  \author mack
 */

class Exporter {
	std::string filename;

public:
	Exporter(std::string);
	virtual ~Exporter();

	std::string getFilename();

	//write FPFHs from PointCloud to file
	void writeFile(PointCloud&);

	//write label files from region growing cluster vector to file
	void writeFile(std::vector<PointCloud*>&);

	//if feature file already exists, delete it so it can be replaced
	void deleteOldFile();
};

#endif /* EXPORTER_H_ */
