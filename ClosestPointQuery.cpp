#include "ClosestPointQuery.h"

//so we can use the VSA package
namespace VSA = CGAL::Surface_mesh_approximation;

ClosestPointQuery::ClosestPointQuery( Mesh& mesh){
    
    std::vector<Kernel::Point_3> anchors; // anchor points 
    std::vector<std::array<std::size_t, 3> > triangles; // indexed triangles
    
    // to make thing simpler, I represent a mesh as an array of triangles.
    // this function takes in a mesh object and constructs and array of triangle indexes.
    // thus each face in our mesh would now be a triangle. 
    VSA::approximate_triangle_mesh(mesh.get_mesh(),
                                    CGAL::parameters::verbose_level(VSA::MAIN_STEPS).
                                    max_number_of_proxies(200).
                                    anchors(std::back_inserter(anchors)). 
                                    triangles(std::back_inserter(triangles))); 
    
    std::cout << "triangle array was created with the size : " << triangles.size() << std::endl;

    //Construct the triangle vector
    for(auto tri : triangles)
    {
        //triangle vertices;
       const K::Point_3 point0(tri[0],0.0, 0.0);
       const K::Point_3 point1(0.0,tri[1],0.0);
       const K::Point_3 point2(0.0,0.0,tri[2]); 

        //construct an array of triangles
        m_triangle_points.push_back(Triangle(point0,point1,point2));
        
    }
}

void ClosestPointQuery::Closest_Single_face(Triangle colided_face, K::Point_3 point, K::Point_3 &closest_point, double &best_distance, int index) const
{
    /* 3 cases of closest points
         1) closest point is on the face of the mesh.
         2) closest point is on an edge of the mesh.
         3) the point that we are given is on the mesh face itself thus the closest point would be itself   
    */ 

    //we use the first vertex of the face (index = 0), inortder to project the query point on the plane of the triangle 
    K::Point_3 first_vertex = colided_face[index]; 
    K::Vector_3 norm = normal(colided_face[0],colided_face[1],colided_face[2]);
    K::Vector_3 projection = norm *((first_vertex - point) * norm);
    
    //we then calculate its squared_distance to see if the projection of the query point lies on the face plane
    double plane_distance_square = projection.squared_length();

    //the closest point does not lie on this face i.e query point is too far from the face. 
    if(plane_distance_square > best_distance) return;

    //query point is in range so we project it on the plane of the triangle 
    K::Point_3 projected = point + projection;
    int outside_count = 0;

    //we loop over the edges of the triangle 
    for(int i = 0; i < 3; i++)
    {
        //edge of the triangle
        K::Point_3 v1 = colided_face[i];
        K::Point_3 v2 = colided_face[ (i+1) % 3];

        //not sure how to conver from point to vec
        K::Vector_3 v1_vec(v1[0],v1[1],v1[2]);
        K::Vector_3 v2_vec(v2[0],v2[1],v2[2]);
        
        //check to see if the projected point is outside of an edge
        K::Point_3 v1_project(v1[0] - projected[0],v1[1] - projected[1],v1[2] - projected[2]);
        K::Point_3 v2_project(v2[0] - projected[0],v2[1] - projected[1],v2[2] - projected[2]);

        //if the point is on the inner sisde of the edge, the dot porduct of normals would point the same way i.e positive norm
        auto outside  = normal(v1_project , v2_project , projected) * norm ;
        if(outside < 0.0)
        {   
            //now we want to calculate the closest point between the edge of the triangle and the query point.
            outside_count++;
            K::Vector_3 edge_dist = v1 - v2; 

            // std::cout << "we are in the outter edge" << std::endl;

            //we want to clamp the projected query point between the edge of the triangle and calculate its closest distance 
            //we only want to do this when we are on the outter edge of the triangle
            double projected_face_normal = (double) ((edge_dist * (projected - v1)) / edge_dist.squared_length());
            double edge_clamp = std::clamp(projected_face_normal , 0.0, 1.0);
            
            //compute the new closest point and distance
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

    /* our point is on the surface of the mesh thus the closest point is itslef */
    if(outside_count == 0)
    {
        // the point is on the surfacce
        std::cout << "the point is on the mesh" << std::endl;
        best_distance = plane_distance_square;
    }
}

K::Point_3 ClosestPointQuery::operator() (K::Point_3 point, float maxDistance) const
{

    //make a copy of triangle_points since function is a const
    std::vector<Triangle> triangle_points_c = m_triangle_points;
    //construct the AABB tree inorder to get the mesh vertex artery
    CGAL::AABB_tree<AABB_triangle_traits> tree(triangle_points_c.begin() , triangle_points_c.end() );
    
    // for our first guess we use the nearest vertex to our point and calcute its distance to the vertex on the mesh 
    K::Point_3 closest_p = tree.closest_point(point);
    double best_distance = K::Vector_3(point, closest_p).squared_length();
    double best_distance_sqrt = std::sqrt(best_distance);
    
    //we check to see if our vertex is in the search distance 
    if(best_distance_sqrt > maxDistance)
    {
        //we are too far from the face just return 0. 
        std::cout<<"We are not within the search distance" << std::endl; 
        return K::Point_3(0,0,0);
    }

    // if we are in the search distance, we construct a box around our point which that is  2*length of nearest vertex
    K::Vector_3 half_diag(best_distance_sqrt,best_distance_sqrt,best_distance_sqrt);
    auto lower_d = point - half_diag;
    auto upper_d = point + half_diag;
    std::vector<Triangle> candidate_trials;
    Box b(lower_d.x() , lower_d.y() ,lower_d.z(), upper_d.x() , upper_d.y(),upper_d.z());


    //since we are initial guess is on the nearest vertex on the mesh, our nearest POINT must be within the faces that colide with our box
    std::list<Tree::Primitive_id> primitives;
    std::vector<Triangle> mesh_colided;
    tree.all_intersected_primitives(b , std::back_inserter(primitives));
    
    //populate the array of the colided meshes
    for(auto prims : primitives)
    {
        mesh_colided.push_back(*prims);

    }
    std::cout << mesh_colided.size() << " faces colided with our box" << std::endl;

    //now for all the meshes we colided with, we want to calculate the which point on the mesh we are closest to. 
    for(auto tris : mesh_colided)
    {
            Closest_Single_face(tris, point, closest_p, best_distance_sqrt, 0);

    }
    std::cout << "the best distance is " << best_distance_sqrt << std::endl;
    std::cout << "the best point is " << closest_p << std::endl;

    return closest_p;
}

