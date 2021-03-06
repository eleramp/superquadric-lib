
/******************************************************************************
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation; either version 2 of the License, or (at your option) any later
* version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.                                                                     *
 ******************************************************************************/

 /**
  * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
  */
  
#ifndef POINTSVTK_H
#define POINTSVTK_H

#include <SuperquadricLibVis/vis.h>

#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkProperty.h>


namespace SuperqVis {

/**
* \class SuperqVis::PointCloud
* \headerfile pointCloud.h <SuperquadricVis/include/pointCloud.h>
*
* \brief A class from SuperqVis namespace.
*
* This class implements a VTK visualization of 3D points.
*/
class PointsVis : public Object
{
protected:
    vtkSmartPointer<vtkPoints> vtk_points;
    vtkSmartPointer<vtkUnsignedCharArray> vtk_colors;
    vtkSmartPointer<vtkPolyData> vtk_polydata;
    vtkSmartPointer<vtkVertexGlyphFilter> vtk_glyphFilter;
public:
    /**
    * Constructor
    */
    PointsVis(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points, const int &point_size);

    /**
    * Destructory
    */
    ~PointsVis() { }

    /**
     * Set the points to visualize
     * @param points is a vector of 3d or 6d Eigen vectors of points
     */
    void set_points(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points);

    /**
     * Set the color of the points for visualization
     * @param colors is a vector of vector of unsigned chars
     * @return true/false if the number of colors is the same as points
     */
    bool set_colors(const std::vector<std::vector<unsigned char>> &colors);

    vtkSmartPointer<vtkPolyData> &get_polydata();
};

}

#endif
