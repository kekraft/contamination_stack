/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/exceptions.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/combine_grids.h>
#include <occupancy_grid_utils/geometry.h>
#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/UInt16MultiArray.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/assign.hpp>
#include <cmath>

namespace gm=geometry_msgs;
namespace nm=nav_msgs;
namespace gu=occupancy_grid_utils;
namespace sm=sensor_msgs;

using std::ostream;
using gu::Cell;
using std::abs;
using boost::bind;
using std::vector;
using std::set;
using boost::assign::operator+=;
using std::operator<<;

const double PI = 3.14159265;

const double TOL=1e-6;


typedef vector<Cell> Path;
typedef set<Cell> Cells;
typedef boost::shared_ptr<nm::OccupancyGrid> GridPtr;
typedef boost::shared_ptr<nm::OccupancyGrid const> GridConstPtr;

nm::MapMetaData contam_map_data;
ros::Subscriber cont_added_sub;
ros::Subscriber cont_map_sub;
ros::Publisher added_cont_cells_pub;
bool map_received = false;


bool samePath (const Path& p1, const Path& p2)
{
  if (p1.size()!=p2.size())
    return false;
  for (unsigned i=0; i<p1.size(); i++)
    if (!(p1[i]==p2[i]))
      return false;
  return true;
}

void setOccupied (nm::OccupancyGrid* g, const unsigned x, const unsigned y)
{
  g->data[gu::cellIndex(g->info, Cell(x, y))] = gu::OCCUPIED;;
}

void setOccupied (GridPtr g, const unsigned x, const unsigned y)
{
  setOccupied(g.get(), x, y);
}

template <class T>
bool equalSets (const set<T>& s1, const set<T>& s2)
{
  BOOST_FOREACH (const T& x, s1) 
  {
    if (s2.find(x)==s2.end())
      return false;
  }
  BOOST_FOREACH (const T& x, s2)
  {
    if (s1.find(x)==s1.end())
      return false;
  }
  return true;
}

bool approxEqual (const double x, const double y)
{
  return abs(x-y)<TOL;
}


namespace geometry_msgs
{

bool operator== (const Polygon& p1, const Polygon& p2)
{
  if (p1.points.size() != p2.points.size())
    return false;
  for (unsigned i=0; i<p1.points.size(); i++) 
    if (!approxEqual(p1.points[i].x, p2.points[i].x) ||
        !approxEqual(p1.points[i].y, p2.points[i].y) || 
        !approxEqual(p1.points[i].z, p2.points[i].z))
      return false;
  return true;  
}

} // namespace geometry_msgs


template <class T>
ostream& operator<< (ostream& str, const vector<T>& s)
{
  str << "(";
  BOOST_FOREACH (const T& x, s) 
    str << x << " ";
  str << ")";
  return str;
}

template <class T>
ostream& operator<< (ostream& str, const set<T>& s)
{
  str << "{";
  std::ostream_iterator<T> iter(str, ", ");
  copy(s.begin(), s.end(), iter);
  str << "}";
  return str;
}


ostream& operator<< (ostream& str, const gm::Point& p)
{
  str << "(" << p.x << ", " << p.y << ")";
  return str;
}

gm::Point makePoint (const double x, const double y)
{
  gm::Point p;
  p.x = x;
  p.y = y;
  return p;
}

gm::Point32 makePoint32 (const double x, const double y)
{
  gm::Point32 p;
  p.x = x;
  p.y = y;
  return p;
}

double norm (const double x, const double y)
{
  return sqrt(x*x + y*y);
}

struct CloseTo
{
  CloseTo(double res) : res(res) {}
  bool operator() (const gm::Point& p, const gm::Point& p2) 
  {
    const double dx=p2.x-p.x;
    const double dy=p2.y-p.y;
    return norm(dx, dy) < res*sqrt(2);
  }
  const double res;
};

double angle (const double x1, const double y1, const double x2, const double y2)
{
  const double ip = x1*x2 + y1*y2;
  return acos(ip/(norm(x1, y1)*norm(x2, y2)));
}

gm::Pose makePose (const double x, const double y, const double theta)
{
  gm::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = tf::createQuaternionMsgFromYaw(theta);
  return p;
}

float dist (const gu::DistanceField& d, int x, int y)
{
  return d[gu::Cell(x,y)];
}

bool pred (const Cell& c)
{
  if (c.x==2 || c.y==2)
    return false;
  if (((c.x==1) || (c.x==3)) &&
      ((c.y==1) || (c.y==3)))
    return false;
  return true;
}

bool not_pred (const Cell& c)
{
  return !pred(c);
}

void test_flow(){

  nm::MapMetaData info;
  info.origin = makePose(.1,.1,0);
  info.resolution = .10;
  info.height = 5;
  info.width = 5;

  gm::Polygon poly;
  poly.points.push_back(makePoint32(.41, .61));
  poly.points.push_back(makePoint32(.43, .45));
  poly.points.push_back(makePoint32(.51, .41));
  poly.points.push_back(makePoint32(.8, .46));
  poly.points.push_back(makePoint32(.78, .63));
  
  const Cells cells = gu::cellsInConvexPolygon(info, poly);
  Cells expected;
  expected.insert(Cell(4,4));
  expected.insert(Cell(4,3));
  expected.insert(Cell(3,4));
  expected.insert(Cell(3,3));
  //EXPECT_EQ(expected, cells);
  for (std::set<Cell>::iterator it = cells.begin(); it != cells.end(); ++it)
  {
      std::cout << " " << *it << std::endl;
  }
  

}

void addContamPolyCB(const geometry_msgs::PolygonStamped::ConstPtr& msg){

  // msg->header.frame_id;
  // msg->header.stamp;

  gm::Polygon poly;
  poly = msg->polygon;
  const Cells cells = gu::cellsInConvexPolygon(contam_map_data, poly);

  // std_msgs::UInt16MultiArray m;
  gm::Polygon cells_poly;

  // m.layout.dim.push_back(std_msgs::MultiArrayDimension());
  // m.layout.dim[0].size = cells.size(); // however many cells there are
  // m.layout.dim[0].stride = cells.size() * 2; // num of cells by 2 dimesnions for cell indices 
  // m.layout.dim[0].label = "cell";
  // m.layout.dim.push_back(std_msgs::MultiArrayDimension());
  // m.layout.dim[0].size = 2; // x and y value
  // m.layout.dim[0].stride = 2;
  // m.layout.dim[0].label = "indices"; // x and y indices of the cell in the grid map


  std::stringstream ss;
  ss << "Polygon received.";
  unsigned int i = 0;
  for (std::set<Cell>::iterator it = cells.begin(); it != cells.end(); ++it)
  {
      ss << " " << *it << std::endl;
      std::cout << "Cell " << *it << std::endl;

      gu::coord_t x = it->x;
      gu::coord_t y = it->y;
      // std::vector<unsigned short> cell_indices;
      gm::Point32 index;
      index.x = x;
      index.y = y;
      index.z = 0;
      cells_poly.points.push_back(index);
      i++;
      
      // cell_indices.push_back(index);
      // cell_indices.push_back((unsigned short)y);
      // unsigned short cell_indices [2] = {(unsigned short)x, (unsigned short)y};
      // m.data.push_back(cell_indices);
      // m.data[i, 0] = x;
      // m.data[i, 1] = y;
      // gm::Point32 
      
  }

  std::cout << ss.str() << std::endl;
  added_cont_cells_pub.publish(cells_poly);



}


void contamGridCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  // Get the map meta data for use in determing cells in polygons
  // Could potentially use the get map service
  if (!map_received) {
   contam_map_data = msg->info; 
   map_received = true;
  }
  // std::cout << "Map received." << std::endl;

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "contam_area_to_cells");

  std::cout << "Compiled and run" << std::endl;

  ros::NodeHandle n;

  cont_added_sub = n.subscribe("/added_contamination_polygon", 10, addContamPolyCB);
  cont_map_sub = n.subscribe("/contamination_grid", 10, contamGridCB);
  added_cont_cells_pub = n.advertise<gm::Polygon>("/added_contamination_cells", 10);

  ros::spin();

  return 0;
}