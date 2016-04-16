/*
An RVIZ GUI tool for adding contamination to the contamination occupancy grid.

The user selects the tool and clicks on the scene. A square mesh is added to the RVIZ scene.

The points are then sent over the topic: /contamination_addition
Using the current rviz tf id as the header for the points.

Based largely on the tutorial for RVIZ GUI Tools found here:
http://docs.ros.org/indigo/api/rviz_plugin_tutorials/html/tool_plugin_tutorial.html
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/load_resource.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/view_controller.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>

 #include <sstream>

#include "add_contamination_tool.h"

namespace contamination_monitor
{

// BEGIN_TUTORIAL
// Construction and destruction
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
//
// Here we set the "shortcut_key_" member variable defined in the
// superclass to declare which key will activate the tool.
AddContaminationTool::AddContaminationTool()
  : moving_rect_node_( NULL )
  , current_rect_property_( NULL )
{
  shortcut_key_ = 'r';

  int sel_start_x_ = -999;
  int sel_start_y_ = -999;
  // icon 
  // setIcon( loadPixmap("package://rviz/icons/classes/MoveCamera.png") );

  topic_property_ = new rviz::StringProperty("Topic", "/contamination_addition", 
                                            "The topic on which to publish points",
                                              getPropertyContainer(), SLOT( updatePointTopic() ), this);

  updatePointTopic();

  contam_topic_property_ = new rviz::StringProperty("Topic", "/added_contamination_polygon", 
                                             "The topic on which to publish the square of contaminated areas.",
                                              getPropertyContainer(), SLOT( updateContamAreaTopic() ), this);

  updateContamAreaTopic();
}

// The destructor destroys the Ogre scene nodes for the rects so they
// disappear from the 3D scene.  The destructor for a Tool subclass is
// only called when the tool is removed from the toolbar with the "-"
// button.
AddContaminationTool::~AddContaminationTool()
{
  for( unsigned i = 0; i < rect_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( rect_nodes_[ i ]);
  }
}

// onInitialize() is called by the superclass after scene_manager_ and
// context_ are set.  It should be called only once per instantiation.
// This is where most one-time initialization work should be done.
// onInitialize() is called during initial instantiation of the tool
// object.  At this point the tool has not been activated yet, so any
// scene objects created should be invisible or disconnected from the
// scene at this point.
//
// In this case we load a mesh object with the shape and appearance of
// the rect, create an Ogre::SceneNode for the moving rect, and then
// set it invisible.
void AddContaminationTool::onInitialize()
{
  rect_resource_ = "package://contamination_monitor/extra/rectangle/model.dae";

  if( rviz::loadMeshFromResource( rect_resource_ ).isNull() )
  {
    ROS_ERROR( "AddContaminationTool: failed to load model resource '%s'.", rect_resource_.c_str() );
    return;
  }

  moving_rect_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( rect_resource_ );
  moving_rect_node_->attachObject( entity );
  moving_rect_node_->setVisible( false );
}

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
//
// First we set the moving rect node to be visible, then we create an
// rviz::VectorProperty to show the user the position of the rect.
// Unlike rviz::Display, rviz::Tool is not a subclass of
// rviz::Property, so when we want to add a tool property we need to
// get the parent container with getPropertyContainer() and add it to
// that.
//
// We wouldn't have to set current_rect_property_ to be read-only, but
// if it were writable the rect should really change position when the
// user edits the property.  This is a fine idea, and is possible, but
// is left as an exercise for the reader.
void AddContaminationTool::activate()
{
  if( moving_rect_node_ )
  {
    moving_rect_node_->setVisible( true );

    current_rect_property_ = new rviz::VectorProperty( "Rect " + QString::number( rect_nodes_.size() ));
    current_rect_property_->setReadOnly( true );
    getPropertyContainer()->addChild( current_rect_property_ );
  }
}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
//
// We make the moving rect invisible, then delete the current rect
// property.  Deleting a property also removes it from its parent
// property, so that doesn't need to be done in a separate step.  If
// we didn't delete it here, it would stay in the list of rects when
// we switch to another tool.
void AddContaminationTool::deactivate()
{
  if( moving_rect_node_ )
  {
    moving_rect_node_->setVisible( false );
    delete current_rect_property_;
    current_rect_property_ = NULL;
  }
}

// Handling mouse events
// ^^^^^^^^^^^^^^^^^^^^^
//
// processMouseEvent() is sort of the main function of a Tool, because
// mouse interactions are the point of Tools.
//
// We use the utility function rviz::getPointOnPlaneFromWindowXY() to
// see where on the ground plane the user's mouse is pointing, then
// move the moving rect to that point and update the VectorProperty.
//
// If this mouse event was a left button press, we want to save the
// current rect location.  Therefore we make a new rect at the same
// place and drop the pointer to the VectorProperty.  Dropping the
// pointer means when the tool is deactivated the VectorProperty won't
// be deleted, which is what we want.
int AddContaminationTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  // Publish point tool code...
  int rects = 0;

  Ogre::Vector3 pos;
  bool success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, pos );
  // setCursor( success ? hit_cursor_ : std_cursor_ );


  std::ostringstream s;
  s << "<b>Left-Click:</b> Select this point.";
  s.precision(3);
  s << " [" << pos.x << "," << pos.y << "," << pos.z << "]";
  setStatus( s.str().c_str() );


  if( !moving_rect_node_ )
  {
    return Render;
  }
  
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection ))
  {
    moving_rect_node_->setVisible( true );
    moving_rect_node_->setPosition( intersection );
    current_rect_property_->setVector( intersection );

    if( event.leftDown() )
    {
      // makeContamRect( intersection );
      current_rect_property_ = NULL; // Drop the reference so that deactivate() won't remove it.

      std::cout << s.str().c_str() << std::endl;

      geometry_msgs::PointStamped ps;
      ps.point.x = pos.x;
      ps.point.y = pos.y;
      ps.point.z = pos.z;
      ps.header.frame_id = context_->getFixedFrame().toStdString();
      ps.header.stamp = ros::Time::now();
      point_pub.publish( ps );

      // Get the dimensions of the rectangle
      std::vector<Ogre::Vector3> rect_pts = getRectDimensions(pos);
      for( unsigned i = 0; i < rect_pts.size(); i++ )
      {
        Ogre::Vector3 pt = rect_pts[ i ];
        
        std::ostringstream dim_str;
        dim_str.precision(3);
        dim_str << " [" << pt.x << "," << pt.y << "," << pt.z << "]";
        std::cout << dim_str.str().c_str() << std::endl;
      }

      // Publish the area of the rectangle
      geometry_msgs::PolygonStamped ply_stmpd;
      
      for( unsigned i = 0; i < rect_pts.size(); i++ )
      {
        Ogre::Vector3 v3 = rect_pts[ i ];
        geometry_msgs::Point32 pt;
        pt.x = v3.x;
        pt.y = v3.y;
        pt.z = v3.z;
        ply_stmpd.polygon.points.push_back( pt ); 
       
      }

      ply_stmpd.header.frame_id = context_->getFixedFrame().toStdString();
      ply_stmpd.header.stamp = ros::Time::now();
      contam_area_pub.publish( ply_stmpd );


      return Render | Finished;
    }
  }
  else
  {
    moving_rect_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the rect.
  }

  return Render;
}

// This is a helper function to create a new rect in the Ogre scene and save its scene node in a list.
void AddContaminationTool::makeContamRect( const Ogre::Vector3& position )
{
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( rect_resource_ );
  node->attachObject( entity );
  node->setVisible( true );
  node->setPosition( position );
  rect_nodes_.push_back( node );
}

// Loading and saving the rects
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// Tools with a fixed set of Property objects representing adjustable
// parameters are typically just created in the tool's constructor and
// added to the Property container (getPropertyContainer()).  In that
// case, the Tool subclass does not need to override load() and save()
// because the default behavior is to read all the Properties in the
// container from the Config object.
//
// Here however, we have a list of named rect positions of unknown
// length, so we need to implement save() and load() ourselves.
//
// We first save the class ID to the config object so the
// rviz::ToolManager will know what to instantiate when the config
// file is read back in.
void AddContaminationTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );

  // The top level of this tool's Config is a map, but our rects
  // should go in a list, since they may or may not have unique keys.
  // Therefore we make a child of the map (``rects_config``) to store
  // the list.
  rviz::Config rects_config = config.mapMakeChild( "Rects" );

  // To read the positions and names of the rects, we loop over the
  // the children of our Property container:
  rviz::Property* container = getPropertyContainer();
  int num_children = container->numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    rviz::Property* position_prop = container->childAt( i );
    // For each Property, we create a new Config object representing a
    // single rect and append it to the Config list.
    rviz::Config rect_config = rects_config.listAppendNew();
    // Into the rect's config we store its name:
    rect_config.mapSetValue( "Name", position_prop->getName() );
    // ... and its position.
    position_prop->save( rect_config );
  }
}

// In a tool's load() function, we don't need to read its class
// because that has already been read and used to instantiate the
// object before this can have been called.
void AddContaminationTool::load( const rviz::Config& config )
{
  // Here we get the "Rects" sub-config from the tool config and loop over its entries:
  rviz::Config rects_config = config.mapGetChild( "Rects" );
  int num_rects = rects_config.listLength();
  for( int i = 0; i < num_rects; i++ )
  {
    rviz::Config rect_config = rects_config.listChildAt( i );
    // At this point each ``rect_config`` represents a single rect.
    //
    // Here we provide a default name in case the name is not in the config file for some reason:
    QString name = "Rect " + QString::number( i + 1 );
    // Then we use the convenience function mapGetString() to read the
    // name from ``rect_config`` if it is there.  (If no "Name" entry
    // were present it would return false, but we don't care about
    // that because we have already set a default.)
    rect_config.mapGetString( "Name", &name );
    // Given the name we can create an rviz::VectorProperty to display the position:
    rviz::VectorProperty* prop = new rviz::VectorProperty( name );
    // Then we just tell the property to read its contents from the config, and we've read all the data.
    prop->load( rect_config );
    // We finish each rect by marking it read-only (as discussed
    // above), adding it to the property container, and finally making
    // an actual visible rect object in the 3D scene at the correct
    // position.
    prop->setReadOnly( true );
    getPropertyContainer()->addChild( prop );
    makeContamRect( prop->getVector() );
  }
}



void AddContaminationTool::updatePointTopic()
{
  point_pub = nh_.advertise<geometry_msgs::PointStamped>( topic_property_->getStdString(), 1 );
}

void AddContaminationTool::updateContamAreaTopic()
{
  contam_area_pub = nh_.advertise<geometry_msgs::PolygonStamped>( contam_topic_property_->getStdString(), 1 );
}

std::vector<Ogre::Vector3> AddContaminationTool::getRectDimensions(Ogre::Vector3 pos){
  std::cout << "Getting dimensions" << std::endl;

  // The rectangle, at this point, is just a square that is .25X.25X.25X.25
  //

  double x1, x2, x3, x4;
  double y1, y2, y3, y4;

  // make square, lowerleft, top left, top right, low right
  x1 =  pos.x; y1 =  pos.y; // both pos
  x2 =  pos.x; y2 =  pos.y + 0.5; // y neg
  x3 =  pos.x + 0.5; y3 =  pos.y + 0.5; // both neg
  x4 =  pos.x + 0.5; y4 =  pos.y; // x neg

  std::vector<Ogre::Vector3> rect_pts;
  rect_pts.push_back(Ogre::Vector3(x1, y1, pos.z));
  rect_pts.push_back(Ogre::Vector3(x2, y2, pos.z));
  rect_pts.push_back(Ogre::Vector3(x3, y3, pos.z));
  rect_pts.push_back(Ogre::Vector3(x4, y4, pos.z));
  // rect_pts.push_back(Ogre::Vector3(x1, y1, pos.z));

  return rect_pts;
}

// End of .cpp file
// ^^^^^^^^^^^^^^^^
//
// At the end of every plugin class implementation, we end the
// namespace and then tell pluginlib about the class.  It is important
// to do this in global scope, outside our package's namespace.

} // end namespace contamination_monitor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(contamination_monitor::AddContaminationTool,rviz::Tool )
