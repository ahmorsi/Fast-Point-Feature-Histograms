#include "Importer.h"
#include <iostream>
#include <fstream>
#include <stdexcept>

Importer::Importer(std::string fn) :
		filename(fn) {
}

Importer::~Importer() {
}

std::string Importer::getFilename() {
	return filename;
}

//read pointcloud from file, if the filename includes "stem" the points are associated with label -1, for "berries" with 1 and 0 otherwise
void Importer::readFile(PointCloud& cloud) {
	std::ifstream infile(filename.c_str());

	if (!infile.good()) {
		throw std::invalid_argument(
				"There's something wrong with the source file at " + filename);
	}

	int l;
	double a, b, c;
	if (filename.find("stem") != std::string::npos) {
		l = -1;
	} else if (filename.find("berries") != std::string::npos) {
		l = 1;
	} else {
		l = 0;
	}
	while (infile >> a >> b >> c) {
		cloud.addPoint(new Point(l, a, b, c));
	}
}

int Importer::readFileWithPrediction(std::string predictionFile,
		int startAtIndex, PointCloud& cloud) {
	std::ifstream infile(filename.c_str());
	std::ifstream predfile(predictionFile.c_str());

	if (!infile.good()) {
		throw std::invalid_argument(
				"There's something wrong with the source file at " + filename);
	}

	if (!predfile.good()) {
		throw std::invalid_argument(
				"There's something wrong with the source file at "
						+ predictionFile);
	}

	int l;
	double ll;
	double a, b, c;

	int count = 0;
	while (count < startAtIndex) {
		predfile >> ll;
		count++;
	}

	int pos = 0;
	int neg = 0;

	while (infile >> a >> b >> c) {
		predfile >> ll;
		count++;
		if (ll > 0) {
			l = 1;
			pos++;
		} else if (ll < 0) {
			l = -1;
			neg++;
		} else {
			l = 0;
		}
		cloud.addPoint(new Point(l, a, b, c));
	}

	infile.close();
	predfile.close();
	return count;
}
