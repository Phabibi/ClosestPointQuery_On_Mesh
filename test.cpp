#include "ClosestPointQuery.cpp"

int main(int argc, char* argv[])
{
  srand (time(NULL));
  /* single point */ 
  std::string path = "../data/sphere.off"; 
  Mesh mesh(path.data());
  K::Point_3 point(1000.0 ,2000.0, 3000.0);
  ClosestPointQuery cp(mesh);
  K::Point_3 closest = cp(point,2.0);

   /* query multiple random points [200.0 , 500.0] */
  // for(int i = 0 ; i < 1000 ; i++)
  // {
  //   float rand_p_0 = 200.0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(500.0-200.0)));
  //   float rand_p_1 = 200.0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(500.0-200.0)));
  //   float rand_p_2 = 200.0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(500.0-200.0)));

  //   K::Point_3 point_ran(rand_p_0 ,rand_p_1, rand_p_2);
  //   K::Point_3 closest = cp(point_ran,2000.0);
  // }

  return 0;
};