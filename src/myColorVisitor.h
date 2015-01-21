#pragma once
#include <osg/array>
#include <osg/geode>
#include <osg/Geometry>
#include <osg/NodeVisitor>
#include <osg/Vec4>
 
 
class  myColorVisitor : public osg::NodeVisitor {
public :
    myColorVisitor();
	myColorVisitor( const osg::Vec4 &color );
    virtual ~myColorVisitor(){};
    virtual void apply ( osg::Node &node ){ traverse( node ); }
    virtual void apply( osg::Geode &geode );
    void setColor( const float r, const float g, const float b, const float a = 1.0f ){
    // -------------------------------------------------------------------
    //
    // Set the color to change apply to the nodes geometry
    //
    // -------------------------------------------------------------------
 
        osg::Vec4 *c = &m_colorArrays->operator []( 0 );
 
        m_color.set( r,g,b,a );
        
        *c = m_color;
 
 
       } // setColor( r,g,b,a )
 
 
    void
    setColor( const osg::Vec4 &color  ){
    // -------------------------------------------------------------------
    //
    // Set the color to change apply to the nodes geometry
    //
    // -------------------------------------------------------------------
 
        osg::Vec4 *c = &m_colorArrays->operator []( 0 );
 
        m_color = color;
        
        *c = m_color;
 
 
       } // setColor( vec4 )
 
 
 
 
 
private :
 
 
    osg::Vec4 m_color;
    osg::ref_ptr< osg::Vec4Array > m_colorArrays;
 
 
 }; // class myColorVisitor

