#include <iostream>
#include <fstream>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include "mesh.h"

typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Vector_3 Vector;
typedef K::Ray_3 Ray;
typedef CGAL::Surface_mesh<Point> Meshs;
typedef boost::graph_traits<Meshs>::face_descriptor face_descriptor;
typedef boost::graph_traits<Meshs>::halfedge_descriptor halfedge_descriptor;
typedef CGAL::AABB_face_graph_triangle_primitive<Meshs> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;


class Tree{

public:
    Tree(Mesh &mesh)
    {
        CGAL::AABB_tree<Traits> c_tree(faces(mesh.get_mesh()).first,faces(mesh.get_mesh()).second, mesh.get_mesh());\
        //use the default assignment operator 
    };
    ~Tree(){}
private:

    //AABB tree object
    CGAL::AABB_tree<Traits> tree;

};