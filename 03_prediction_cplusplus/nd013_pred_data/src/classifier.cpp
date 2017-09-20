#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include "classifier.h"

using namespace std;

/**
 * Initializes GNB
 */
GNB::GNB() {

}

GNB::~GNB() {}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{
	/*
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d, 
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
	*/
	int feature_size = data[0].size();

	unique_labels_ = labels;
	vector<string>::iterator sit;
	std::sort(unique_labels_.begin(), unique_labels_.end());
	sit = std::unique(unique_labels_.begin(), unique_labels_.end());
	unique_labels_.erase(sit, unique_labels_.end());

	map<string, vector<vector<double> > > table_of_features;
	for(auto label: unique_labels_) {
		label_counts_[label] = 0;
		vector<double> tmp(feature_size, 0.0);
		means_.insert(std::pair<string, vector<double> >(label,tmp));
		stds_.insert(std::pair<string, vector<double> >(label,tmp));
	}

	// Collect data - class count , mean
	int train_size = labels.size();
	for(auto i = 0; i < train_size; i++) {
		label_counts_[labels[i]] +=1;
		table_of_features[labels[i]].push_back(data[i]);
		for(auto j = 0; j < feature_size; j++) {
			means_[labels[i]][j] += data[i][j];
		}
	}

	// Calculate the mean
	for (auto label: unique_labels_) {
		p_label_[label] = label_counts_[label] * 1.0 / labels.size(); // prob of label (or class)
	
		for (auto j = 0; j < feature_size; j++) {   // for each feature
		  means_[label][j]  /= label_counts_[label];  // MEAN
		}
	}

	// Calculate the variance
	for(auto label: unique_labels_) {
		for(auto j = 0; j < feature_size; j++) {
			vector<double> diff_sq(4, 0.0);
			for(auto row = 0; row < table_of_features[label].size(); row++) {
				stds_[label][j] += pow(table_of_features[label][row][j] - means_[label][j], 2.0);
			}
			stds_[label][j] /= label_counts_[label];
			stds_[label][j] = sqrt(stds_[label][j]);
		}
	}



	cout << "Labels: " << endl;

	for(auto label: unique_labels_) {

		cout << label << " has samples: " << label_counts_[label] << endl;

		cout << "Mean: ";
		for(const auto &j: means_[label]) {
			cout << j << ", ";
		}
		cout << endl;

		cout << "Std: ";
		for(const auto &k: stds_[label]) {
			cout << k << ", ";
		}

		cout << endl;

		cout << "Prob: ";
		cout << p_label_[label] << endl;


	}

	

}

string GNB::predict(vector<double> sample)
{
	/*
		Once trained, this method is called and expected to return 
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
	*/

	map<string, double> p;
	double max = 0;
	string result;

	for(auto label: unique_labels_) {
		p[label] = p_label_[label];
		for(auto j = 0; j < sample.size(); j++) {
			double norm = 1.0 / sqrt( 2 * M_1_PI * pow(stds_[label][j], 2));
			double num = pow(sample[j] - means_[label][j], 2);
			double denom = 2 * pow(stds_[label][j],2);
			p[label] *= norm * exp(-num / denom);
		}
		if(max < p[label]){
			max = p[label];
			result = label;
		}
	}

	return result;

}
