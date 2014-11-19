#include "Background.h"

#define DISTANCE_FROM_PLANE(a,b,c,d,x0,y0,z0) (a*x0+b*y0+c*z0+d)/sqrt(a*a+b*b+c*c)
Background::Background(){
	for(int i=0;i<NO_PLANES;i++){
		pcl::ModelCoefficients::Ptr dummy(new pcl::ModelCoefficients);
		coefficients.push_back(dummy);
	}
}
Background::Background(bool forceRetrain, std::string path, int start, int end){
	_path = path;
	_start = start;
	_end = end;

	for(int i=0;i<NO_PLANES;i++){
		pcl::ModelCoefficients::Ptr dummy(new pcl::ModelCoefficients);
		coefficients.push_back(dummy);
	}
	
	bgInfoFile = new QFile("BackgroundInfo.dat");
	if(!bgInfoFile->exists() || forceRetrain){
		//train mode
		char num[5];
		sprintf(num,"%03d",(end-start)/2);
		std::string filename = path+std::string(num);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename,*cloud) == -1){
			std::cerr<<"FILE DOES NOT EXIST"<<std::endl;
			return;
		}
		trainBackground(cloud);
		//test_background(path,start,end);
	}
	else{
		//load training file
		load_background();
		//test_background(path,start,end);
	}
	trained = true;
}


Background::~Background(void){

}

void Background::trainBackground(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
	
	//pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	seg.setInputCloud (cloud);
	// Extract the inliers
	extract.setInputCloud (cloud);
	extract.setNegative (true);

	for(int i=0;i<NO_PLANES;i++){
		seg.segment (*inliers, *(coefficients[i]));

		extract.setIndices (inliers);
		extract.filter (*cloud);
	}
	save_background();
}

void Background::removeBackground(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud){

	//for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->begin();it!=cloud->end();it++){
	//	for(int j=0;j<NO_PLANES;j++){
	//		if( DISTANCE_FROM_PLANE(coefficients[j]->values[0],coefficients[j]->values[1],coefficients[j]->values[2],
	//								coefficients[j]->values[3],it->x,it->y,it->z) < 0.01){
	//									cloud->erase(it);
	//									it--;
	//									break;
	//		}
	//	}
	//}

	//pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	seg.setInputCloud (cloud);
	// Extract the inliers
	extract.setInputCloud (cloud);
	extract.setNegative (true);

	for(int i=0;i<NO_PLANES;i++){
		seg.segment (*inliers, *(coefficients[i]));

		extract.setIndices (inliers);
		extract.filter(*cloud);
		remove_noise(cloud,coefficients[i]);
	}
}

void Background::save_background(){
	std::ofstream outfile (bgInfoFile->fileName().toStdString(),std::ofstream::out); 
	for(int i=0;i<NO_PLANES;i++){
		for(int j=0;j<4;j++)
			outfile<<coefficients[i]->values[j]<<std::endl;
	}
	outfile.close();
}

void Background::load_background(){
	std::ifstream infile (bgInfoFile->fileName().toStdString(),std::ofstream::in); 
	for(int i=0;i<NO_PLANES;i++){
		coefficients[i]->values.resize(4);
		for(int j=0;j<4;j++)
			infile>>coefficients[i]->values[j];
	}
	infile.close();
}

void Background::test_background(std::string path, int start, int end){

	for(int i=start;i<=end;i++){
		char num[5];
		sprintf(num,"%03d",i);
		std::string filename = path+std::string(num);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename,*cloud) == -1){
			std::cerr<<"FILE DOES NOT EXIST"<<std::endl;
			return;
		}
		float cloudpoints = cloud->size();
		removeBackground(cloud);
		std::cout<<filename<<": "<<float(cloud->size()) / cloudpoints<<std::endl;
	}
}

void Background::remove_noise(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud,pcl::ModelCoefficients::Ptr coefficients){
	static int count = 1;
	for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->begin();it!=cloud->end();it++){
			double dist = abs(coefficients->values[0]*it->x +
								coefficients->values[1]*it->y + 
								coefficients->values[2]*it->z + 
								coefficients->values[3])/
								sqrt(coefficients->values[0]*coefficients->values[0] + 
								coefficients->values[1]*coefficients->values[1] +
								coefficients->values[2]*coefficients->values[2]);
			if((count % 2 == 1 && dist < 0.01) || (count % 2 == 0 && dist < 0.1)){
				cloud->erase(it);
				it--;
			}
		}
	count++;
}


