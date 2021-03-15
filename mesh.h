#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <fstream>
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::Point_3                    Point;


/* abstract mesh class that encapsulates CGAL::Surface_mesh<point> */ 

class Mesh
{
    public:
        // default constructor to make a mesh from an .OFF file
        Mesh(const std::string path){
            std::ifstream input(path.data());
            if(!input || !(input >> mesh) || mesh.is_empty())
            {
                std::cerr << "not a valid OFF file" << std::endl;
                return ;
            }
            std::cout << "mesh object successfully read with "<< mesh.number_of_faces() << " faces " << std::endl; 

        }
        
        ~Mesh(){};
        CGAL::Surface_mesh<Point> get_mesh(){return mesh;}

    private:
        //mesh object
        CGAL::Surface_mesh<Point> mesh;

};