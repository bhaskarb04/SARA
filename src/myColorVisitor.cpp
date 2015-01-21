#include "myColorVisitor.h"

myColorVisitor::myColorVisitor() : NodeVisitor( NodeVisitor::TRAVERSE_ALL_CHILDREN ) {
	m_color.set( 1.0, 1.0, 1.0, 1.0 );
 
	m_colorArrays = new osg::Vec4Array;
 
	m_colorArrays->push_back( m_color );    
}

myColorVisitor::myColorVisitor( const osg::Vec4 &color ) : NodeVisitor( NodeVisitor::TRAVERSE_ALL_CHILDREN ) {
 
	m_color = m_color;
 
	m_colorArrays = new osg::Vec4Array;
 
	m_colorArrays->push_back( m_color );    
 
}

void myColorVisitor::apply( osg::Geode &geode ){
	osg::StateSet *state   = NULL;
	unsigned int    vertNum = 0;
	unsigned int numGeoms = geode.getNumDrawables();
	
 
	for( unsigned int geodeIdx = 0; geodeIdx < numGeoms; geodeIdx++ ) {
		osg::Geometry *curGeom = geode.getDrawable( geodeIdx )->asGeometry();
		if ( curGeom ) {
			osg::Vec4Array *colorArrays = dynamic_cast< osg::Vec4Array *>(curGeom->getColorArray());
			if ( colorArrays ) {  
				for ( unsigned int i = 0; i < colorArrays->size(); i++ ) { 
					osg::Vec4 *color = &colorArrays->operator [](i);
					color->set( m_color._v[0], m_color._v[1], m_color._v[2], m_color._v[3]);                   
				}
			}   
			else{            
				curGeom->setColorArray( m_colorArrays.get());
				curGeom->setColorBinding( osg::Geometry::BIND_OVERALL );            
			}
			curGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		}
	}
        
}  
    
    
    
 