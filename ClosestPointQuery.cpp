#include "ClosestPointQuery.h"

//so we can use the VSA package
namespace VSA = CGAL::Surface_mesh_approximation;

void ClosestPointQuery::Closest_Single_face(Triangle colided_face, K::Point_3 point, K::Point_3 &closest_point, double &best_distance, int index)
{
    //project the point from the norm
    K::Point_3 first_vertex = colided_face[index]; 
    K::Vector_3 norm = normal(colided_face[0],colided_face[1],colided_face[2]);
    K::Vector_3 projection = norm *((first_vertex - point) * norm);
    int outside_count = 0;
    
    //check to see if the plane distance squared is 
    double plane_distance_square = projection.squared_length();
    //we already have our best distance
    if(plane_distance_square > best_distance) return;

    K::Point_3 projected = point + projection;

    for(int i = 0; i < 3; i++)
    {
        K::Point_3 v1 = colided_face[i];
        K::Point_3 v2 = colided_face[ (i+1) % 3];

        //not sure how to conver from point to vec
        K::Vector_3 v1_vec(v1[0],v1[1],v1[2]);
        K::Vector_3 v2_vec(v2[0],v2[1],v2[2]);
        
        //check to see if we are outside of an edge

        K::Point_3 v1_project(v1[0] - projected[0],v1[1] - projected[1],v1[2] - projected[2]);
        K::Point_3 v2_project(v2[0] - projected[0],v2[1] - projected[1],v2[2] - projected[2]);

        bool outside  = normal(v1_project , v2_project , projected) * norm < 0.0;
        if(outside)
        {   outside_count++;
            K::Vector_3 edge_dist = v1 - v2; 
            // std::cout << "we are in the outter edge" << std::endl;
            //we wanna clamp the projection on the edge and calculate its distance with our best guess
            //not we only want to do this when we are on the outter edge of the triangle

            double projected_face_normal = (double) ((edge_dist * (projected - v1)) / edge_dist.squared_length());
            double edge_clamp = std::clamp(projected_face_normal , 0.0, 1.0);
            
            //compute the new closest point
            K::Vector_3 new_closest = v2_vec * edge_clamp + v1_vec *(1.0-edge_clamp);
            K::Point_3 new_point = point - new_closest;
            K::Vector_3 new_point_vec(new_point[0] - point[0], new_closest[1] - point[1], new_closest[2] - point[2]);
            
            double new_closest_dist = new_point_vec.squared_length();

            //check to see if we changed our distance
            if(new_closest_dist < best_distance)
            {
                // std::cout << "we got a better distance at : " << edge_dist << std::endl;
                //update point and distance
                best_distance = new_closest_dist;
                closest_point = new_point;
            }

        }

    }

    if(outside_count == 0)
    {
        // the point is on the surfacce
        std::cout << "the point is on the mesh" << std::endl;
        best_distance =    plane_distance_square;
    }
}

ClosestPointQuery::ClosestPointQuery( K::Point_3 point,  Mesh& mesh,float maxDistance ){
    //convert the mesh into an array of triangles.
    
    std::vector<Kernel::Point_3> anchors;
    std::vector<std::array<std::size_t, 3> > triangles;
    std::vector<Triangle> triangle_points;
    // free function interface with named parameters
    VSA::approximate_triangle_mesh(mesh.get_mesh(),
                                    CGAL::parameters::verbose_level(VSA::MAIN_STEPS).
                                    max_number_of_proxies(200).
                                    anchors(std::back_inserter(anchors)). // anchor points
                                    triangles(std::back_inserter(triangles))); // indexed triangles
    std::cout << "triangle array was created with the size : " << triangles.size() << std::endl;
    for(auto tri : triangles)
    {
        //triangle vertecies;
       const K::Point_3 point0(tri[0],0.0, 0.0);
       const K::Point_3 point1(0.0,tri[1],0.0);
       const K::Point_3 point2(0.0,0.0,tri[2]); 

        triangle_points.push_back(Triangle(point0,point1,point2));
        
    }

    //construct the AABB tree
    CGAL::AABB_tree<AABB_triangle_traits> tree(triangle_points.begin() , triangle_points.end() );
    K::Point_3 closest_p = tree.closest_point(point);

    //best square distance first iteration
    double best_distance = K::Vector_3(point, closest_p).squared_length();
    double best_distance_sqrt = std::sqrt(best_distance);
    
    if(best_distance_sqrt > maxDistance)
    {
        //we are too far from the face
        std::cout<<"We are not within the search distance" << std::endl; 
    }

    //construct the box diagonals within the search distance
    K::Vector_3 half_diag(best_distance_sqrt,best_distance_sqrt,best_distance_sqrt);
    auto lower_d = point - half_diag;
    auto upper_d = point + half_diag;

    //construct a box that is point+bext_distance_square * point+best_distance_Squared wide.
    //this will be our collision distance box
    std::vector<Triangle> candidate_trials;
    Box b(lower_d.x() , lower_d.y() ,lower_d.z(), upper_d.x() , upper_d.y(),upper_d.z());
    std::list<Tree::Primitive_id> primitives;

    //all the colided meshes with our box
    std::vector<Triangle> mesh_colided;
    tree.all_intersected_primitives(b , std::back_inserter(primitives));
    for(auto prims : primitives)
    {
        mesh_colided.push_back(*prims);

    }
    for(auto tris : mesh_colided)
    {
            this->Closest_Single_face(tris, point, closest_p, best_distance_sqrt, 1);

    }
    std::cout << "the best distance is " << best_distance_sqrt << std::endl;
    std::cout << "the best point is " << closest_p << std::endl;


}

