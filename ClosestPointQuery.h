/* STL */ 
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstdint>
#include <iomanip>

/* CGAL */ 
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh_approximation/approximate_triangle_mesh.h>
#include <CGAL/AABB_triangle_primitive.h>

/* Mesh */
#include "mesh.h"

typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Triangle_3 Triangle;
typedef CGAL::Bbox_3 Box;
typedef std::vector<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;




class ClosestPointQuery
{
public:
/* Constructs a triangluated mesh and calculates the closest distance to a given point within a search distance */ 
ClosestPointQuery( Mesh& mesh);
/* empty destructor */
~ClosestPointQuery(){};
/* Calculates the closest single face on a given face */ 
void Closest_Single_face(Triangle colided_face, K::Point_3 point, K::Point_3 &closest_point, double &best_distance, int index);

/* calculates the closest distance to a given point within a search distance for all triangles */
void Closest_face(K::Point_3 point, float maxDistance);

/* A mesh of Triangles */ 
std::vector<Triangle> triangle_points;
// Point operator() (const Point& queryPoint, float maxDist) const;
};