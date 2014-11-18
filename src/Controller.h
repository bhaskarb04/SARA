#pragma once
#ifndef _CONTROLLER_
#define _CONTROLLER_

#include "src/HandModel.h"
#include "PointCloudViewer.h"
#include "Background.h"

class Controller
{
	bool show_background;
	bool show_manipulator_raw;
	bool show_manipulator_processed;
	bool show_blocks_raw;
	bool show_blocks_processed;
	int _bgstart, _bgend, _manstart, _manend, _realstart, _realend;
	pointCloudViewer *pclviewer;
	std::string pathToData;

	Background* _background;
	HandModel* _handmodel;

public:
	Controller(void);
	~Controller(void);
	void setPath(std::string path,int bgstart,int bgend,int manstart,int manend,int realstart,int realend);
	void showFull(bool);
	void showFullProcessed(bool);
	void startShow();
	void showImmediate(std::string path,int start,int end);
	void record();
	void trainManipulator();
	void trainBackground();
	void process(bool removebg,bool removemanip,bool show);
};

#endif