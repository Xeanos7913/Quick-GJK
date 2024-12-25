
# Quick GJK

A simple 3D collision detector and contact point generator for convex shapes using the infamous GJK algorithm. It's written in plain c++ and relies on GLM for mathematics.

## Video Demo:

https://youtu.be/PWnCpF387UY

## READ THIS:

If you look at the internal code, you'll notice a GameObject* owner inside the Collider struct. This NEEDS to be replaced by your own Entity from your ECS or whatever you're using. Basically, it needs the owner's transform matrix for the mathematics it does.

The project does NOT support any concave meshes. It has been tested using concave meshes loaded in from .obj files.

## Usage

    1. The createPolytopeFromPoints(std::vector<glm::vec3>& points, std::vector<int> indices) function is the main Collider creator function. I tested it with the vertices and indices loaded in from an obj file using tinyobjloader.h, and it worked for pretty much any convex mesh.

    2. The updatePolytope() function is important for updating the collider per frame using its owner's transform matrix.

    3. The result of the collision is returned inside the gCollisionResult struct. It also has some useless things inside it which you can remove if you'd like. I kept it there because too lazy.

    4. That's pretty much it. The code itself is quite heavily commented, so it should be quite self-explanatory.

## Inspiration

The base of the algorithm is based on KevinMoran's implementation of the GJK and EPA algorithm: https://github.com/kevinmoran/GJK

## Further Development

I'm currently in the process of developing a collision detector that can work with concave meshes and be parallalizable as well, so we can run it on the GPU for blazing speeds. That implementation will be having a few dependencies though.
