#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include "project/Importer.h"

/**
 * Implementation of histogram classification using svm-light.
 *
 * \author: mack
 */

int main(int argc, char **argv) {
	//TODO: use svm light training and classifying methods
	//model file has to be stored in "data/model" and named "model.dat",
	//predicition file in "data/prediction" and named "prediction.dat"


	char* trainFile = "data/histograms/train/histograms.dat";
	char* testFile = "data/histograms/test/histograms.dat";
	char* modelFile = "data/model/model.dat";
	char* predicationFile = "data/prediction/prediction.dat";

	for (int i = 1; i < argc; ++i) {
		if (strcmp(argv[i], "-help") == 0) {
			std::cout << "The following parameters are available:  \n"
					<< "-Tr: The path of Histogram Training File (default: data/histograms/train/histograms.dat) \n"
					<< "-Te: The path of Histogram Testing File (default: data/histograms/test/histograms.dat) \n"
					<< "-M: The path of the generated Model File (default: data/model/model.dat) \n"
					<< "-P: The path of the generated predication File (default: data/prediction/prediction.dat) \n"
					<< std::endl;
			return 1;
		}
		if (strcmp(argv[i], "-Tr") == 0) {
			trainFile = argv[++i];
		} else if (strcmp(argv[i], "-Te") == 0) {
			testFile = argv[++i];
		} else if (strcmp(argv[i], "-M") == 0) {
			modelFile = argv[++i];
		} else if (strcmp(argv[i], "-P") == 0) {
			predicationFile = argv[++i];
		}
	}

	char trainCmd[100];
	char classifyCmd[100];
	sprintf(trainCmd,"./svm_light/svm_learn %s %s",trainFile,modelFile);
	system(trainCmd);
	sprintf(classifyCmd,"./svm_light/svm_classify %s %s %s",testFile,modelFile,predicationFile);
	system(classifyCmd);
}
