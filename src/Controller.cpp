#include "Controller.h"


Controller::Controller(void){
	show_background = false;
	show_manipulator_raw = false;
	show_manipulator_processed = false;
	show_blocks_raw = false;
	show_blocks_processed = false;
	pclviewer = new pointCloudViewer;
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
		if(removemanip)
			_handmodel->removeHand(cloud);
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