#pragma once
#ifndef _BACKGROUND_
#define _BACKGROUND_


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl\io\pcd_io.h>
#include <pcl\filters\conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl\ModelCoefficients.h>
#include <pcl\point_types.h>

#include <pcl\filters\extract_indices.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\features\normal_3d.h>
#include <pcl\kdtree\kdtree.h>
#include <pcl\segmentation\extract_clusters.h>

#include <qfile.h>

#define NO_PLANES 2
class Background
{
	bool trained;
	QFile* bgInfoFile;
	std::string _path;
	int _start;
	int _end;
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	std::vector<pcl::ModelCoefficients::Ptr> coefficients;

	void trainBackground(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
	void save_background();
	void load_background();
	void test_background(std::string path, int start, int end);
public:
	Background(bool forceRetrain, std::string path, int start, int end);
	~Background(void);
	void removeBackground(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
	inline bool isTrained(){return trained;}
};

#endif