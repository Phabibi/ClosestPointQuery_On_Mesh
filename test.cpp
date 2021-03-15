#include "ClosestPointQuery.cpp"

int main(int argc, char* argv[])
{
  std::string path = "./sphere.off"; 
  Mesh mesh(path.data());
  K::Point_3 point(100.0 ,200.0, 300.0);
  ClosestPointQuery cp(mesh);
  cp.Closest_face(point,200.0);
  return 0;
};