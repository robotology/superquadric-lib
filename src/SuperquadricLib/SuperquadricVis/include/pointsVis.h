#ifndef POINTSVTK_H
#define POINTSVTK_H

#include "vis.h"

#include <vector>
#include <Eigen/Dense>
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
    PointsVis(const std::vector<Eigen::Vector3d> &points, const int point_size);

    /**
    * Destructory
    */
    ~PointsVis() { }

    /**
     * Set the points to visualize
     * @param points is a vector of 3d or 6d Eigen vectors of points
     */
    void set_points(const std::vector<Eigen::Vector3d> &points);

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
