#pragma once
#include <osg\NodeCallback>
#include <osg\Group>
#include <opencv2\opencv.hpp>
class imgprocesscallback :	public osg::NodeCallback {
public:
	imgprocesscallback(cv::Mat* frame,cv::Mat* depthframe,osg::Group* root);
	~imgprocesscallback(void);

protected:
	cv::Mat*	_frame;
	cv::Mat*	_depthframe;
	cv::Mat		_prevframe;
	cv::Mat		_bgframe;
	osg::Group*	_root;

private:
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
};

