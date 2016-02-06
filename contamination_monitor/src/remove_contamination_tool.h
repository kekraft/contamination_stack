/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef ADD_CONTAMINATION_TOOL_H
#define ADD_CONTAMINATION_TOOL_H

#include <rviz/tool.h>

# include <ros/node_handle.h>
# include <ros/publisher.h>

namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class StringProperty;
class BoolProperty;
}

namespace contamination_monitor
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Tool.  Every tool
// which can be added to the tool bar is a subclass of
// rviz::Tool.
class RemoveContaminationTool: public rviz::Tool
{
Q_OBJECT
public:
  RemoveContaminationTool();
  ~RemoveContaminationTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

private:
  void makeContamRect( const Ogre::Vector3& position );
  std::vector<Ogre::Vector3> getRectDimensions( const Ogre::Vector3);

  std::vector<Ogre::SceneNode*> rect_nodes_;
  Ogre::SceneNode* moving_rect_node_;
  std::string rect_resource_;
  rviz::VectorProperty* current_rect_property_;

public Q_SLOTS:

  void updatePointTopic();

  void updateContamAreaTopic();

protected:
  ros::NodeHandle nh_;
  ros::Publisher point_pub;
  ros::Publisher contam_area_pub;

  
  rviz::StringProperty* topic_property_;
  rviz::BoolProperty* auto_deactivate_property_;

  rviz::StringProperty* contam_topic_property_;

};
// END_TUTORIAL

} // end namespace contamination_monitor

#endif // ADD_CONTAMINATION_TOOL_H