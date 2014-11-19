#include "HandModel.h"


HandModel::HandModel(){
	load_model();
	trained = true;
}
HandModel::HandModel(std::string base,int start, int end,bool show){

	pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer("Record show");
	viewer->setBackgroundColor (0, 0, 0);
	int count = 0;
	for(int i=start;i<=end;i++){
		char num[5];
		sprintf(num,"%03d",i);
		std::string str = base+std::string(num);
		std::string fname = base+std::string("SkinPoints_")+std::string(num);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(str,*cloud) == -1){
			std::cerr<<"Cannot load file"<<endl;
			return;
		}

		std::vector<int> nindices;
		pcl::removeNaNFromPointCloud<pcl::PointXYZRGBA>(*cloud,*cloud,nindices);
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		viewer->addPlane(*removeplane(cloud),"plane1");

		pcl::ModelCoefficients::Ptr coefficients = removeplane(cloud);
		viewer->addPlane(*coefficients,"plane2");

		remove_noise(cloud,coefficients);

		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
		//viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");

		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
		tree->setInputCloud (cloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
		ec.setClusterTolerance (0.02); // 2cm
		ec.setMinClusterSize (1000);
		ec.setMaxClusterSize (50000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud);
		ec.extract (cluster_indices);
		int j=0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (),j<1; ++it,++j)
		{
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				cloud_cluster->points.push_back (cloud->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_cluster);
			viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_cluster, rgb, "sample cloud");
			pcl::io::savePCDFileBinary<pcl::PointXYZRGBA>(fname,*cloud_cluster);
			pcl::PointXYZRGBA p1,p2,p3;
			pcl::getMinMax3D(*cloud_cluster,p1,p2);
			p3.x = (p1.x + p2.x)/2;
			p3.y = (p1.y + p2.y)/2;
			p3.z = (p1.z + p2.z)/2;
			double rad = pcl::euclideanDistance(p1,p2)/2;
			viewer->addSphere(p3,rad);
	
		}
		//if(i!=start && count - cloud->size() < -10000)
			//continue;
		viewer->spinOnce(100);
		boost::this_thread::sleep( boost::posix_time::microseconds(10000));

		cout<<"Cloud size: "<<cloud->size()<<endl;
		count = cloud->size();
	}
}


HandModel::~HandModel(void){
}

pcl::ModelCoefficients::Ptr HandModel::removeplane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud){

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
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
	seg.segment (*inliers, *coefficients);
	/*if(coefficients->values.size()){
		std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
											<< coefficients->values[1] << " "
											<< coefficients->values[2] << " " 
											<< coefficients->values[3] << std::endl;
	}*/

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	// Extract the inliers
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*cloud);

	return (coefficients);
}

void HandModel::remove_noise(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::ModelCoefficients::Ptr coefficients){
	
	for(int i=0;i<cloud->size();i++){
			double dist = abs(coefficients->values[0]*cloud->at(i).x +
								coefficients->values[1]*cloud->at(i).y + 
								coefficients->values[2]*cloud->at(i).z + 
								coefficients->values[3])/
								sqrt(coefficients->values[0]*coefficients->values[0] + 
								coefficients->values[1]*coefficients->values[1] +
								coefficients->values[2]*coefficients->values[2]);
			if(dist < 0.1){
				cloud->erase(cloud->begin()+i);
				i--;
			}
		}
}

HandModel::HandModel(vector<std::string> filelist){
	vector<cv::Point3f> skinvals;
	for(vector<std::string>::iterator it=filelist.begin(); it!=filelist.end();it++){
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(*it,*cloud) == -1){
			std::cerr<<"Cannot load file"<<endl;
			return;
		}
		for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator cit=cloud->begin();cit!=cloud->end();cit++){
			double citsum=cit->r+cit->g+cit->b;
			skinvals.push_back(cv::Point3f(cit->r/citsum,cit->g/citsum,cit->b/citsum));
		}
	}
	
	skinfunction_1 *sf1 = new skinfunction_1;
	sf1->skintrain(skinvals);
	model.push_back(sf1);
	/*skinfunction_2 *sf2 = new skinfunction_2;
	sf2->skintrain(skinvals);
	model.push_back(sf2);*/

}

void HandModel::save_model(){
	for(int i=0;i<model.size();i++){
		model[i]->save_function();
	}
}

void HandModel::load_model(){

	//Function 1
	skinfunction_1 *sf1 = new skinfunction_1;
	sf1->load_function();
	model.push_back(sf1);
}

void HandModel::removeHand(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud){
	skincount = 0;
	for(int i=0;i<model.size();i++){
		for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->begin();it!=cloud->end();it++){
			double ssum = it->r+it->g+it->b;
			if(model[i]->skincompare(it->r/ssum,it->g/ssum,it->b/ssum)){
				skincount++;
				cloud->erase(it);
				it--;
			}
		}
	}
	cout<<skincount<<endl;
}