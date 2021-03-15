# ClosestPointQuery On Mesh
The following API is an implementation of the closest point query on a given mesh. The ClosestPointQuery class takes in a Mesh object and constructs an array of triangles or faces which would then be used to calculate the closest point on a mesh with a given point. It is assumed that the input mesh is of type .OFF and the surface mesh is triangulated.

The test.cpp contains some tests with multiple points on the mesh. 
The [Sphere.off](https://pasteboard.co/JSNdWgY.png) is a triangulated mesh file. 


## Dependencies 
The main dependancy

```CGAL```:  https://www.cgal.org/download 

```CMAKE```: https://cmake.org/download/
