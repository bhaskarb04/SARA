#pragma warning( disable : 4996 )
#include "Controller.h"

#define VISUALIZE_ONLY

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                        void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;
  }
}

void visualize_only(std::string base,int start, int end){
	pointCloudViewer *pclviewer = new pointCloudViewer;
#ifdef VISUALIZE_ONLY
	//pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer("Record show");
	//viewer->setBackgroundColor (0, 0, 0);
	
#endif
	//std::ifstream file;
	//file.open("../Data/info.data",std::ios_base::in);
	skinfunction_1 skinmodel;
	skinmodel.load_function();
	while (!pclviewer->done()){
		for(int i=start;i<=end;i++){
			int x;
			//file>>x;
			//if(!x)
			//	continue;
			cout<<i<<endl;
			char num[5];
			sprintf(num,"%03d",i);
			std::string fname = base+std::string(num);//+std::string("_Points.dat");
			std::string fname2 = base+std::string("processed")+std::string(num);//+std::string("_Points.dat");

			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
		
#ifndef VISUALIZE_ONLY		
			if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fname,*cloud) == -1){
				std::cerr<<"Cannot load file"<<endl;
				return;
			}


			for(pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->begin();it!=cloud->end();it++){
				double ssum = it->r+it->g+it->b;
				if(skinmodel.skincompare(it->r/ssum,it->g/ssum,it->b/ssum)){
					cloud->erase(it);
					it--;
				}
			}
			pcl::io::savePCDFileBinary<pcl::PointXYZRGBA>(fname2,*cloud);
#endif

#ifdef VISUALIZE_ONLY
			if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fname,*cloud) == -1){
				std::cerr<<"Cannot load file"<<endl;
				return;
			}
			/*viewer->removeAllPointClouds();
			viewer->addPointCloud(cloud);
			viewer->spinOnce(100);*/
			pclviewer->removeClouds();
			pclviewer->addPointCloud(cloud);
			pclviewer->frame();
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
#endif
		}
	}
}



int main (){
	
	Controller c;
	c.setPath("../Data/Full1/CloudRGBAll_",0,35,41,86,96,160);
	//c.showFull(true);
	//c.startShow();
	//c.showFullProcessed(true);
	//c.startShow();
	c.trainBackground();
	c.trainManipulator();
	c.process(true,true,false);
	c.showImmediate("../Data/Full1/CloudRGBAll_PROCESSED",96,160);
	return 0;
	visualize_only("../Data/Full1/CloudRGBAll_",0,190);
	return 0;

#ifdef RECORD
	SimpleOpenNIViewer v;
	v.run ();
	return 0;
#else
	//Analyzer analyzer("../Data/CloudRGBHandOnly_020"); //CloudRGBHandOnly_060,080,20
#endif
	//HandModel hm(std::string("../Data/All/Hand/CloudRGBAll_"),41,86,false);
	/*vector<std::string> files;
	for(int i=41;i<=86;i++){
		char num[5];
		sprintf(num,"%03d",i);
		files.push_back(std::string("../Data/All/Hand/CloudRGBAll_SkinPoints_")+std::string(num));
	}
	HandModel hm(files);
	hm.save_model();*/
}