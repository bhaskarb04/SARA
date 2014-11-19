#include <osg/ShapeDrawable>
#include <osg/PolygonMode>

#include "Controller.h"


Controller::Controller(void){
	show_background = false;
	show_manipulator_raw = false;
	show_manipulator_processed = false;
	show_blocks_raw = false;
	show_blocks_processed = false;
	pclviewer = new pointCloudViewer;
	_background = NULL;
	_handmodel = NULL;
}


Controller::~Controller(void)
{
	delete pclviewer;
}

void Controller::setPath(std::string path,int bgstart,int bgend,int manstart,int manend,int realstart,int realend){
	pathToData = path;
	_bgstart = bgstart;
	_bgend = bgend;
	_manstart = manstart;
	_manend = manend;
	_realstart = realstart;
	_realend = realend;
}

void Controller::showFull(bool option){
	if(option){
		show_background = true;
		show_manipulator_raw = true;
		show_manipulator_processed = false;
		show_blocks_raw = true;
		show_blocks_processed = false;
	}
}

void Controller::showFullProcessed(bool option){
	if(option){
		show_background = true;
		show_manipulator_raw = true;
		show_manipulator_processed = false;
		show_blocks_raw = false;
		show_blocks_processed = true;
	}
}

void Controller::startShow(){
	int level = 0;
	while (!pclviewer->done()){
		int start,end;
		std::string processed="";
		switch(level){
		case(0):
			if(show_background){
				start = _bgstart;
				end = _bgend;
			}
			level++;
			break;
		case(1):
			if(show_manipulator_raw || show_manipulator_processed){
				start = _manstart;
				end = _manend;
				if(show_manipulator_processed) processed = "processed";
			}
			level++;
			break;
		case(2):
			if(show_blocks_raw || show_blocks_processed){
				start = _realstart;
				end = _realend;
				if(show_blocks_processed) processed = "processed";
			}
			level = 0;
			break;
		}
		for(int i=start;i<=end;i++){
			char num[5];
			sprintf(num,"%03d",i);
			std::string fname = pathToData+processed+std::string(num);//+std::string("_Points.dat");
			cout<<fname<<endl;
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
			if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fname,*cloud) == -1){
				std::cerr<<"Cannot load file"<<endl;
				return;
			}
			pclviewer->removeClouds();
			pclviewer->addPointCloud(cloud);
			pclviewer->frame();
			//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
	}
}

void Controller::record(){

}

void Controller::trainBackground(){
	if(_background == NULL || !_background->isTrained())
	_background = new Background(false,pathToData,_bgstart,_bgend);
}

void Controller::trainManipulator(){
	if(_handmodel == NULL || !_handmodel->isTrained())
		_handmodel = new HandModel();
}

void Controller::process(bool removebg,bool removemanip,bool show){
	for(int i=_realstart;i<=_realend;i++){
		//Load and check the data
		char num[5];
		sprintf(num,"%03d",i);
		std::string fname = pathToData+std::string(num);//+std::string("_Points.dat");
		std::string fname2 = pathToData+"PROCESSED"+std::string(num);//+std::string("_Points.dat");
		cout<<fname<<endl;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fname,*cloud) == -1){
			std::cerr<<"Cannot load file"<<endl;
			return;
		}
		//Remove bg
		if(removebg)
			_background->removeBackground(cloud);
		if(removemanip){
			_handmodel->removeHand(cloud);
			if(_handmodel->isHandPresent())
				std::cout<<"Hand present"<<std::endl;
			else
				std::cout<<"Nothing present"<<std::endl;
		}
		////
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
		tree->setInputCloud (cloud);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
		ec.setClusterTolerance (0.02);
		ec.setMinClusterSize (1000);
		ec.setMaxClusterSize (50000);
		ec.setSearchMethod (tree);
		ec.setInputCloud (cloud);
		ec.extract (cluster_indices);
		cout<<"Clusters: "<<cluster_indices.size()<<endl;
		pclviewer->removeShapes();
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			cout<<"Cluster size: "<<it->indices.size()<<endl;
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				cloud_cluster->points.push_back (cloud->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_cluster);
			//viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_cluster, rgb, "sample cloud");
			//pcl::io::savePCDFileBinary<pcl::PointXYZRGBA>(fname,*cloud_cluster);
			pcl::PointXYZRGBA p1,p2,p3;
			pcl::getMinMax3D(*cloud_cluster,p1,p2);
			p3.x = (p1.x + p2.x)/2;
			p3.y = (p1.y + p2.y)/2;
			p3.z = (p1.z + p2.z)/2;
			double rad = pcl::euclideanDistance(p1,p2)/2;
			//viewer->addSphere(p3,rad);
			osg::Sphere* sphere = new osg::Sphere(osg::Vec3d(p3.x,p3.y,p3.z),rad);
			osg::ShapeDrawable* unitSphereDrawable = new osg::ShapeDrawable(sphere);
			osg::Geode* sphereGeode = new osg::Geode();
			sphereGeode->getOrCreateStateSet()->
				setAttribute( new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE) );
			sphereGeode->addDrawable(unitSphereDrawable);
			pclviewer->addShape(sphereGeode);
		}

		////
		pclviewer->removeClouds();
		pclviewer->addPointCloud(cloud);
		pclviewer->frame();
		//pcl::io::savePCDFileBinary<pcl::PointXYZRGBA>(fname2,*cloud);
	}
}

void Controller::showImmediate(std::string path,int start,int end){
	while (!pclviewer->done()){
		
		for(int i=start;i<=end;i++){
			char num[5];
			sprintf(num,"%03d",i);
			std::string fname = path+std::string(num);//+std::string("_Points.dat");
			//cout<<fname<<endl;
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
			if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fname,*cloud) == -1){
				std::cerr<<"Cannot load file"<<endl;
				return;
			}
			pclviewer->removeClouds();
			pclviewer->addPointCloud(cloud);
			pclviewer->frame();
		}
	}
}