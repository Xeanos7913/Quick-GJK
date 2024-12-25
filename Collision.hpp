#pragma once

#include <vector>
#include <glm.hpp>
#include <memory>
#include <iostream>
#include <algorithm>
#include <gtx/norm.hpp>

struct Collider {
    glm::vec3 pos;            // Origin in world space
    glm::mat3 matRS;          // Rotation/scale component of model matrix
    glm::mat3 matRS_inverse;
    //GameObject* owner;       // you need to plug the actual owner object here, of your own engine.
    virtual glm::vec3 support(glm::vec3 dir) = 0;
};

struct Polytope : Collider {
    float* points;  // (x0 y0 z0 x1 y1 z1 etc)
    int num_points;

    // Each face now has a vector of vertex indices and a normal (vec3)
    std::vector<std::tuple<std::vector<int>, glm::vec3>> faces;

    glm::vec3 support(glm::vec3 dir) override {
        dir = matRS_inverse * dir;

        glm::vec3 furthest_point = glm::vec3(points[0], points[1], points[2]);
        float max_dot = dot(furthest_point, dir);

        for (int i = 3; i < num_points * 3; i += 3) {
            glm::vec3 v = glm::vec3(points[i], points[i + 1], points[i + 2]);
            float d = dot(v, dir);
            if (d > max_dot) {
                max_dot = d;
                furthest_point = v;
            }
        }
        return matRS * furthest_point + pos;  // Convert support to world space
    }
};

// Gift-wrapping (Jarvis March) algorithm to find the convex hull in 2D
std::vector<int> giftWrapConvexHull(const std::vector<glm::vec2>& points) {
    int n = points.size();
    if (n == 3) {
        // If we have exactly 3 points, return them as they form a triangle (convex hull)
        return { 0, 1, 2 };
    }

    std::vector<int> hull;

    // Find the leftmost point
    int leftmost = 0;
    for (int i = 1; i < n; ++i) {
        if (points[i].x < points[leftmost].x) {
            leftmost = i;
        }
    }

    int p = leftmost;
    do {
        hull.push_back(p);
        int q = (p + 1) % n;  // Candidate for the next point in the hull

        for (int r = 0; r < n; ++r) {
            if (r == p) continue; // Skip the current point

            // Calculate the cross product to check the turn direction
            glm::vec2 v1 = points[q] - points[p];
            glm::vec2 v2 = points[r] - points[p];
            float cross = v1.x * v2.y - v1.y * v2.x;

            // If r is more counterclockwise than q, or if they are collinear and r is farther
            if (cross < 0 || (cross == 0 && glm::distance(points[r], points[p]) > glm::distance(points[q], points[p]))) {
                q = r;
            }
        }

        p = q;
    } while (p != leftmost);

    return hull;  // Return the indices of the vertices forming the convex hull
}

// Struct representing a polygonal face with vertices and indices
struct PolygonalFace {
    std::vector<glm::vec3> faceVertices;  // List of vertices
    std::vector<int> faceIndices;

    // Compute the face normal using the first 3 vertices (assumes face is planar)
    glm::vec3 getFaceNormal() const {
        if (faceVertices.size() < 3) return glm::vec3(0.0f);
        glm::vec3 v0 = faceVertices[0];
        glm::vec3 v1 = faceVertices[1];
        glm::vec3 v2 = faceVertices[2];

        return glm::normalize(glm::cross(v1 - v0, v2 - v0));
    }

    // Compute the centroid of the face
    glm::vec3 getCentroid() const {
        glm::vec3 centroid(0.0f);
        for (const auto& vertex : faceVertices) {
            centroid += vertex;
        }
        centroid /= static_cast<float>(faceVertices.size());
        return centroid;
    }

    // Project 3D point onto 2D plane
    glm::vec2 projectOntoPlane(const glm::vec3& point, const glm::vec3& normal, const glm::vec3& u, const glm::vec3& v) const {
        glm::vec3 relativePoint = point - getCentroid();
        return glm::vec2(glm::dot(relativePoint, u), glm::dot(relativePoint, v));
    }

    // Generate the edges using the convex hull
    std::vector<std::pair<glm::vec3, glm::vec3>> getEdges() const {
        std::vector<std::pair<glm::vec3, glm::vec3>> edges;
        int n = faceVertices.size();

        if (n < 3) return edges;  // No edges possible with fewer than 3 points

        // 1. Calculate face normal (used to project points onto a plane)
        glm::vec3 normal = getFaceNormal();

        // 2. Define two orthogonal vectors on the plane
        glm::vec3 u = glm::normalize(faceVertices[1] - faceVertices[0]);
        glm::vec3 v = glm::normalize(glm::cross(normal, u));

        // 3. Project the 3D vertices onto the 2D plane
        std::vector<glm::vec2> projectedPoints;
        for (const auto& vertex : faceVertices) {
            projectedPoints.push_back(projectOntoPlane(vertex, normal, u, v));
        }

        // 4. Compute the convex hull indices using the giftWrapConvexHull algorithm
        std::vector<int> hullIndices = giftWrapConvexHull(projectedPoints);

        // 5. Create edges based on the convex hull indices
        int hullSize = hullIndices.size();
        for (int i = 0; i < hullSize; ++i) {
            glm::vec3 v0 = faceVertices[hullIndices[i]];
            glm::vec3 v1 = faceVertices[hullIndices[(i + 1) % hullSize]];  // Wrap around to form edges
            edges.emplace_back(v0, v1);
        }

        return edges;  // Return the valid convex hull edges
    }
};

// Helper to check if two normals are approximately equal
bool areNormalsEqual(const glm::vec3& n1, const glm::vec3& n2, float epsilon = 0.0001f) {
    return glm::length(n1 - n2) < epsilon;
}

// Helper to check if two faces share an edge
bool shareEdge(const std::vector<int>& face1, const std::vector<int>& face2) {
    int sharedVertices = 0;
    for (int v1 : face1) {
        if (std::find(face2.begin(), face2.end(), v1) != face2.end()) {
            ++sharedVertices;
        }
    }
    return sharedVertices >= 2;
}


// This function has been tested for verts and indices returned by .obj file formats. ONLY WORKS WITH CONVEX MESHES!!! It does NOT convert concave meshes to convex meshes itself!!! 
std::shared_ptr<Polytope> createPolytopeFromPoints(const std::vector<glm::vec3>& points, const std::vector<int>& indices) {
    auto polytope = std::make_shared<Polytope>();

    polytope->num_points = static_cast<int>(points.size());
    polytope->points = new float[polytope->num_points * 3];

    for (int i = 0; i < polytope->num_points; ++i) {
        polytope->points[i * 3] = points[i].x;
        polytope->points[i * 3 + 1] = points[i].y;
        polytope->points[i * 3 + 2] = points[i].z;
    }

    for (size_t i = 0; i < indices.size(); i += 3) {
        if (i + 2 < indices.size()) {
            int idx1 = indices[i];
            int idx2 = indices[i + 1];
            int idx3 = indices[i + 2];

            if (idx1 >= points.size() || idx2 >= points.size() || idx3 >= points.size()) {
                std::cerr << "Error: Invalid index detected at face: " << idx1 << ", " << idx2 << ", " << idx3 << std::endl;
                continue;
            }

            glm::vec3 v1 = points[idx1];
            glm::vec3 v2 = points[idx2];
            glm::vec3 v3 = points[idx3];

            glm::vec3 edge1 = v2 - v1;
            glm::vec3 edge2 = v3 - v1;

            glm::vec3 normal = glm::normalize(glm::cross(edge1, edge2));

            if (glm::length(normal) == 0.0f) {
                std::cerr << "Warning: Degenerate triangle found: " << idx1 << ", " << idx2 << ", " << idx3 << std::endl;
                continue;
            }

            // Attempt to merge with existing faces
            bool merged = false;
            for (size_t faceIndex = 0; faceIndex < polytope->faces.size(); ++faceIndex) {
                auto& face = polytope->faces[faceIndex];
                auto& face_indices = std::get<0>(face);
                auto& face_normal = std::get<1>(face);

                if (areNormalsEqual(face_normal, normal) && shareEdge(face_indices, { idx1, idx2, idx3 })) {
                    // Merge the face by adding new unique vertices
                    face_indices.push_back(idx1);
                    face_indices.push_back(idx2);
                    face_indices.push_back(idx3);
                    // Remove duplicate vertices
                    std::sort(face_indices.begin(), face_indices.end());
                    face_indices.erase(std::unique(face_indices.begin(), face_indices.end()), face_indices.end());

                    // Replace the old face with the merged face
                    polytope->faces[faceIndex] = std::make_tuple(face_indices, normal);

                    merged = true;
                    break;
                }
            }

            if (!merged) {
                // Create a new face if it cannot be merged
                polytope->faces.emplace_back(std::vector<int>{idx1, idx2, idx3}, normal);
            }
        }
    }

    // Remove merged faces from the vector of faces
    std::vector<std::tuple<std::vector<int>, glm::vec3>> uniqueFaces;
    for (auto& face : polytope->faces) {
        auto& face_indices = std::get<0>(face);

        // Check if this face is unique and not merged into another
        if (std::find_if(uniqueFaces.begin(), uniqueFaces.end(), [&](const auto& uniqueFace) {
            return std::get<0>(uniqueFace) == face_indices;
            }) == uniqueFaces.end()) {
            uniqueFaces.push_back(face);
        }
    }
    polytope->faces = std::move(uniqueFaces);

    return polytope;
}

// This is a helper function that relies on the GameObject* owner of this. You need to plug in your own owner object here.

/*
std::vector<PolygonalFace> createPolygonalFaces(const Polytope& polytope) {
    std::vector<PolygonalFace> polygonalFaces;

    // Retrieve the model matrix for transforming local vertices to world space
    glm::mat4 modelMatrix = polytope.owner->getModel();

    for (const auto& face : polytope.faces) {
        const auto& indices = std::get<0>(face);  // Extract indices
        glm::vec3 normal = std::get<1>(face);     // Extract face normal

        PolygonalFace polygonFace;
        polygonFace.faceIndices = indices;

        // Populate faceVertices with transformed coordinates
        for (int index : indices) {
            int pointIdx = index * 3;
            glm::vec3 localVertex(
                polytope.points[pointIdx],
                polytope.points[pointIdx + 1],
                polytope.points[pointIdx + 2]
            );

            // Transform local vertex to world coordinates
            glm::vec4 worldVertex = modelMatrix * glm::vec4(localVertex, 1.0f);
            polygonFace.faceVertices.push_back(glm::vec3(worldVertex));
        }

        polygonalFaces.push_back(polygonFace);
    }

    return polygonalFaces;
}

*/

// Same here: This is the function responsible for updating the collider per frame, according to the Owner gameobject's position, rotation and scale.

/*
void updatePolytope(Polytope& tope) {
    tope.matRS = tope.owner->getModel();
    tope.matRS_inverse = inverse(tope.matRS);
    tope.pos = tope.owner->getPosition();
}

*/


// Useful helper function if you wanna debug if the faces are being returned properly or nah.
std::tuple<std::vector<glm::vec3>, glm::vec3> getFaceVertices(const Polytope& polytope, const std::tuple<std::vector<int>, glm::vec3>& face) {
    const auto& indices = std::get<0>(face);
    std::vector<glm::vec3> vertices;

    for (int idx : indices) {
        vertices.emplace_back(
            polytope.points[idx * 3],
            polytope.points[idx * 3 + 1],
            polytope.points[idx * 3 + 2]
        );
    }

    return std::make_tuple(vertices, std::get<1>(face));  // Return vertices and the normal
}

// Function to extract the vertices of a face by index, transforming them into world coordinates
// EXETREMELY IMPORTANT FUNCTION!!! You need to plug in the actual owner object's transform matrix here:
PolygonalFace getFaceByIndex(const Polytope& polytope, int faceIndex) {
    if (faceIndex < 0 || faceIndex >= polytope.faces.size()) {
        std::cerr << "Error: Face index out of bounds." << std::endl;
        return PolygonalFace();  // Return an empty face if index is invalid
    }

    const auto& face = polytope.faces[faceIndex];  // Get the face by index
    const auto& globalIndices = std::get<0>(face); // Get the global vertex indices of the face

    PolygonalFace polyFace;
    polyFace.faceIndices.clear();  // We will create local indices

    // Get the owner's model matrix to transform local coordinates to world space

    // YOU NEED TO USE YOUR ACTUAL OWNER'S TRANSFORM MATRIX HERE:
    glm::mat4 modelMatrix = glm::mat4x4(1.0f);//polytope.owner->getModel();

    // For each global index, extract the corresponding vertex and transform it into world space
    for (size_t i = 0; i < globalIndices.size(); ++i) {
        int globalIndex = globalIndices[i];

        // Extract the local vertex in polytope space
        glm::vec3 localVertex(
            polytope.points[globalIndex * 3],       // X coordinate
            polytope.points[globalIndex * 3 + 1],   // Y coordinate
            polytope.points[globalIndex * 3 + 2]    // Z coordinate
        );

        // Transform the local vertex into world coordinates using the model matrix
        glm::vec4 worldVertex = modelMatrix * glm::vec4(localVertex, 1.0f);
        polyFace.faceVertices.push_back(glm::vec3(worldVertex));  // Store the transformed vertex

        // Add local index for this face, which ranges from 0 to n-1
        polyFace.faceIndices.push_back(static_cast<int>(i));  // Local index is sequential
    }

    return polyFace;  // Return the PolygonalFace struct with world-space vertices and face-specific indices
}


// Now the helpers are done, GJK, EPA and clipper begins:

constexpr auto GJK_MAX_NUM_ITERATIONS = 64;

// Struct to hold the result of the EPA algorithm
struct gCollisionResult {

    glm::vec3 collisionNormal;
    float penetrationDepth;

    Polytope* referenceCollider;
    std::tuple<std::vector<int>, glm::vec3> referenceFace;

    Polytope* incidentCollider;
    std::tuple<std::vector<int>, glm::vec3> incidentFace;

    PolygonalFace referenceFacce;
    PolygonalFace incidentFacce;

    std::vector<glm::vec3> manifold;
};

//Contact Point storage struct
struct ContactPoint {
    glm::vec3 position;
    glm::vec3 normal;
    float penetrationDepth;
};

// ease of life struct
struct GJKsimplex {
    glm::vec3 a, b, c, d;
    int dimension;
};

bool gjk(Polytope* coll1, Polytope* coll2, gCollisionResult* mtv);
//Internal functions used in the GJK algorithm
void update_simplex3(GJKsimplex& simplex, int& simp_dim, glm::vec3& search_dir);
bool update_simplex4(GJKsimplex& simplex, int& simp_dim, glm::vec3& search_dir);

gCollisionResult EPA(GJKsimplex& simplex, Polytope* coll1, Polytope* coll2);

// Helper function to get a vertex from the points array using an index
glm::vec3 getVertexFromIndex(const Polytope* polytope, int index) {
    return glm::vec3(polytope->points[index * 3], polytope->points[index * 3 + 1], polytope->points[index * 3 + 2]);
}

bool gjk(Polytope* coll1, Polytope* coll2, gCollisionResult* mtv) {

    GJKsimplex simplex{};

    glm::vec3 search_dir = coll1->pos - coll2->pos; // Initial search direction between colliders

    // Get initial point for simplex
    simplex.c = coll2->support(search_dir) - coll1->support(-search_dir);
    search_dir = -simplex.c; // Search in direction of origin

    // Get second point for a line segment simplex
    simplex.b = coll2->support(search_dir) - coll1->support(-search_dir);

    if (glm::dot(simplex.b, search_dir) < 0) {
        // If we didn't reach the origin, no collision
        return false;
    }

    search_dir = glm::cross(glm::cross(simplex.c - simplex.b, -simplex.b), simplex.c - simplex.b); // Search perpendicular to the line segment towards the origin
    if (search_dir == glm::vec3(0, 0, 0)) {
        // If origin is on this line segment, any normal search vector will do
        search_dir = glm::cross(simplex.c - simplex.b, glm::vec3(1, 0, 0)); // Normal with x-axis
        if (search_dir == glm::vec3(0, 0, 0)) {
            search_dir = glm::cross(simplex.c - simplex.b, glm::vec3(0, 0, -1)); // Normal with z-axis
        }
    }

    simplex.dimension = 2; // Simplex dimension

    for (int iterations = 0; iterations < GJK_MAX_NUM_ITERATIONS; iterations++) {
        simplex.a = coll2->support(search_dir) - coll1->support(-search_dir);
        if (glm::dot(simplex.a, search_dir) < 0) {
            // If we didn't reach the origin, no collision
            return false;
        }

        simplex.dimension++;
        if (simplex.dimension == 3) {
            update_simplex3(simplex, simplex.dimension, search_dir);
        }
        else if (update_simplex4(simplex, simplex.dimension, search_dir)) {
            if (mtv) {
                // Call EPA to calculate the full collision result (normal, depth, and point)
                *mtv = EPA(simplex, coll1, coll2);
            }
            return true; // Collision detected
        }
    }

    // No collision detected within iteration limit
    return false;
}

//Triangle case
void update_simplex3(GJKsimplex& simplex, int& simp_dim, glm::vec3& search_dir) {
    glm::vec3 n = cross(simplex.b - simplex.a, simplex.c - simplex.a); //triangle's normal
    glm::vec3 AO = -simplex.a; //direction to origin

    //Determine which feature is closest to origin, make that the new simplex

    simp_dim = 2;
    if (dot(cross(simplex.b - simplex.a, n), AO) > 0) { //Closest to edge AB
        simplex.c = simplex.a;
        search_dir = cross(cross(simplex.b - simplex.a, AO), simplex.b - simplex.a);
        return;
    }
    if (dot(cross(n, simplex.c - simplex.a), AO) > 0) { //Closest to edge AC
        simplex.b = simplex.a;
        search_dir = cross(cross(simplex.c - simplex.a, AO), simplex.c - simplex.a);
        return;
    }

    simp_dim = 3;
    if (dot(n, AO) > 0) { //Above triangle
        simplex.d = simplex.c;
        simplex.c = simplex.b;
        simplex.b = simplex.a;
        search_dir = n;
        return;
    }
    simplex.d = simplex.b;
    simplex.b = simplex.a;
    search_dir = -n;
    return;
}

//Tetrahedral case
bool update_simplex4(GJKsimplex& simplex, int& simp_dim, glm::vec3& search_dir) {

    //Get normals of three new faces
    glm::vec3 ABC = cross(simplex.b - simplex.a, simplex.c - simplex.a);
    glm::vec3 ACD = cross(simplex.c - simplex.a, simplex.d - simplex.a);
    glm::vec3 ADB = cross(simplex.d - simplex.a, simplex.b - simplex.a);

    glm::vec3 AO = -simplex.a; //dir to origin
    simp_dim = 3; //hoisting this just cause

    //Plane-test origin with 3 faces
    if (dot(ABC, AO) > 0) { //In front of ABC
        simplex.d = simplex.c;
        simplex.c = simplex.b;
        simplex.b = simplex.a;
        search_dir = ABC;
        return false;
    }
    if (dot(ACD, AO) > 0) { //In front of ACD
        simplex.b = simplex.a;
        search_dir = ACD;
        return false;
    }
    if (dot(ADB, AO) > 0) { //In front of ADB
        simplex.c = simplex.d;
        simplex.d = simplex.b;
        simplex.b = simplex.a;
        search_dir = ADB;
        return false;
    }

    //else inside tetrahedron; enclosed!
    return true;
}

// Compute the signed distance from a point to a plane
float signedDistanceToPlaneGJK(const glm::vec3& point, const glm::vec3& planeOrigin, const glm::vec3& planeNormal) {
    return glm::dot(point - planeOrigin, planeNormal);
}

// get the witness faces for collision point generation
void findContactManifoldFaces(Polytope* collider1, Polytope* collider2, const glm::vec3& collisionNormal,
    std::tuple<std::vector<int>, glm::vec3>& referenceFace, std::tuple<std::vector<int>, glm::vec3>& incidentFace, int& referenceFaceIndex, int& incidentFaceIndex, int& refOrInci) {

    // Step 1: Transform the collision normal using each collider's inverse matrix
    glm::vec3 transformedNormal1 = collider1->matRS_inverse * collisionNormal;
    glm::vec3 transformedNormal2 = collider2->matRS_inverse * collisionNormal;

    // Step 2: Find the best face on each collider using the transformed normals
    float best_dot1 = -1.0f, best_dot2 = -1.0f;
    int best_index1 = -1, best_index2 = -1;

    // For collider1 (reference collider candidate)
    for (int i = 0; i < collider1->faces.size(); ++i) {
        const auto& face = collider1->faces[i];
        const glm::vec3& normal = std::get<1>(face);  // Access the normal
        float dot_product = glm::dot(glm::normalize(normal), glm::normalize(-transformedNormal1));

        //std::cout << "collider1 normal: " << glm::to_string(normal) << std::endl;

        if (dot_product > best_dot1) {
            best_dot1 = dot_product;
            best_index1 = i;
        }
    }

    // For collider2 (incident collider candidate)
    for (int i = 0; i < collider2->faces.size(); ++i) {
        const auto& face = collider2->faces[i];
        const glm::vec3& normal = std::get<1>(face);  // Access the normal
        float dot_product = glm::dot(glm::normalize(normal), glm::normalize(transformedNormal2));

        //std::cout << "collider2 normal: " << glm::to_string(normal) << std::endl;

        if (dot_product > best_dot2) {
            best_dot2 = dot_product;
            best_index2 = i;
        }
    }

    // Step 3: Determine the reference and incident faces with scale consideration
    Polytope* referenceCollider;
    Polytope* incidentCollider;
    int best_reference_index, best_incident_index;

    const float epsilon = 0.00001f;  // threshold for "nearly identical" dot products

    if (std::abs(best_dot1 - best_dot2) < epsilon) {
        // Dots are nearly identical; choose by comparing scale magnitudes

        // Yeah, once again, plug in your actual owner gameobject here:
        float scaleMagnitude1 = glm::length(collider1->owner->getScale());
        float scaleMagnitude2 = glm::length(collider2->owner->getScale());

        if (scaleMagnitude1 >= scaleMagnitude2) {
            referenceCollider = collider1;
            incidentCollider = collider2;
            best_reference_index = best_index1;
            best_incident_index = best_index2;
            refOrInci = 1;  // collider1 is reference
        }
        else {
            referenceCollider = collider2;
            incidentCollider = collider1;
            best_reference_index = best_index2;
            best_incident_index = best_index1;
            refOrInci = 2;  // collider2 is reference
        }
    }
    else if (best_dot1 > best_dot2) {
        referenceCollider = collider1;
        incidentCollider = collider2;
        best_reference_index = best_index1;
        best_incident_index = best_index2;
        refOrInci = 1;  // collider1 is reference
    }
    else {
        referenceCollider = collider2;
        incidentCollider = collider1;
        best_reference_index = best_index2;
        best_incident_index = best_index1;
        refOrInci = 2;  // collider2 is reference
    }

    referenceFaceIndex = best_reference_index;
    incidentFaceIndex = best_incident_index;
}

// Compute the squared distance between two line segments
float squaredDistanceBetweenEdgesGJK(const glm::vec3& p1, const glm::vec3& q1, const glm::vec3& p2, const glm::vec3& q2) {
    glm::vec3 d1 = q1 - p1;  // Direction vector of segment S1
    glm::vec3 d2 = q2 - p2;  // Direction vector of segment S2
    glm::vec3 r = p1 - p2;

    float a = glm::dot(d1, d1);  // Squared length of segment S1
    float e = glm::dot(d2, d2);  // Squared length of segment S2
    float f = glm::dot(d2, r);

    float s, t;
    if (a <= 1e-6f && e <= 1e-6f) {
        return glm::dot(r, r);  // Both segments degenerate into points
    }
    if (a <= 1e-6f) {
        s = 0.0f;
        t = glm::clamp(f / e, 0.0f, 1.0f); // Closest point on segment S2 to point p1
    }
    else {
        float c = glm::dot(d1, r);
        if (e <= 1e-6f) {
            t = 0.0f;
            s = glm::clamp(-c / a, 0.0f, 1.0f); // Closest point on segment S1 to point p2
        }
        else {
            float b = glm::dot(d1, d2);
            float denom = a * e - b * b;
            if (denom != 0.0f) {
                s = glm::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            }
            else {
                s = 0.0f;
            }
            t = glm::clamp((b * s + f) / e, 0.0f, 1.0f);
        }
    }

    glm::vec3 c1 = p1 + s * d1;
    glm::vec3 c2 = p2 + t * d2;
    return glm::length2(c1 - c2); // Squared distance between the closest points on the segments
}

// Find the closest pair of edges between two colliders
std::vector<std::pair<std::pair<glm::vec3, glm::vec3>, std::pair<glm::vec3, glm::vec3>>> findClosestEdgesGJK(const PolygonalFace& face1, const PolygonalFace& face2) {
    std::vector<std::pair<glm::vec3, glm::vec3>> edges1 = face1.getEdges();
    std::vector<std::pair<glm::vec3, glm::vec3>> edges2 = face2.getEdges();

    // Not the best solution for this! But I was too lazy to implement a better solution. Ideally what we should do is, if we find no edges below this epsilon,
    // we should check for the closest edge. Because the only time we won't find edges within epsilon is when there has been significant overlap, and we need to push the colliders apart!!!
    float minDistanceSquared = 0.01f;  // I'm talking about this epsilon here. It's quite an arbitrary value I just randomly put here, you should see what works best for you.
    std::vector<std::pair<std::pair<glm::vec3, glm::vec3>, std::pair<glm::vec3, glm::vec3>>> closestEdges;

    // Iterate over all edge pairs
    for (const auto& edge1 : edges1) {
        for (const auto& edge2 : edges2) {
            // Compute the squared distance between the two edges
            float distSquared = squaredDistanceBetweenEdgesGJK(edge1.first, edge1.second, edge2.first, edge2.second);

            // Track the closest pair of edges
            if (distSquared < minDistanceSquared) {
                minDistanceSquared = distSquared;
                closestEdges.push_back({ edge1, edge2 });
            }
        }
    }

    return closestEdges;
}

// Compute closest points between two line segments
glm::vec3 closestPointBetweenLinesGJK(const glm::vec3& p1, const glm::vec3& q1, const glm::vec3& p2, const glm::vec3& q2) {
    glm::vec3 d1 = q1 - p1;  // Direction vector of segment S1
    glm::vec3 d2 = q2 - p2;  // Direction vector of segment S2
    glm::vec3 r = p1 - p2;
    float a = glm::dot(d1, d1);  // Squared length of segment S1
    float e = glm::dot(d2, d2);  // Squared length of segment S2
    float f = glm::dot(d2, r);

    float s, t;
    if (a <= 1e-6f && e <= 1e-6f) {
        // Both segments degenerate into points
        return p1;
    }
    if (a <= 1e-6f) {
        // First segment degenerate into a point
        s = 0.0f;
        t = glm::clamp(f / e, 0.0f, 1.0f); // Use the closest point on segment S2 to point p1
    }
    else {
        float c = glm::dot(d1, r);
        if (e <= 1e-6f) {
            // Second segment degenerate into a point
            t = 0.0f;
            s = glm::clamp(-c / a, 0.0f, 1.0f); // Use the closest point on segment S1 to point p2
        }
        else {
            float b = glm::dot(d1, d2);
            float denom = a * e - b * b;

            if (denom != 0.0f) {
                s = glm::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            }
            else {
                s = 0.0f;
            }

            t = glm::clamp((b * s + f) / e, 0.0f, 1.0f);
        }
    }

    // Compute the closest points on the actual segments
    glm::vec3 c1 = p1 + s * d1; // Closest point on segment S1
    glm::vec3 c2 = p2 + t * d2; // Closest point on segment S2

    // Return the point that is on the closest segment line (S1 or S2)
    return glm::length(c1 - p1) < glm::length(c2 - p2) ? c1 : c2;
}

std::vector<glm::vec3> HandleEdgeCollision(const PolygonalFace& incidentFace, const PolygonalFace& referenceFace) {

    //get the edges that are close enough to each other
    auto edges = findClosestEdgesGJK(incidentFace, referenceFace);

    std::vector<glm::vec3> manifoldToReturn;

    for (const auto& edgePair : edges) {
        // store their point of intersection, or closest point into the manifoldToReturn
        auto point = closestPointBetweenLinesGJK(edgePair.first.first, edgePair.first.second, edgePair.second.first, edgePair.second.second);

        manifoldToReturn.push_back(point);
    }

    return manifoldToReturn; // return the manifold
}

// Function to clip a polygonal face (incidentFace) against the planes (polygonal faces) of a reference collider
std::vector<glm::vec3> clipFaceAgainstReference(const PolygonalFace& incidentFace, const Polytope& referenceCollider, const PolygonalFace& ReferenceFace) {
    std::vector<glm::vec3> clippedVertices = incidentFace.faceVertices;
    const float epsilon = 0.001f;  // Tolerance for numerical precision

    // Loop through each face of the reference collider
    for (size_t faceIndex = 0; faceIndex < referenceCollider.faces.size(); ++faceIndex) {
        // Create a PolygonalFace for the current face of the reference collider
        PolygonalFace referenceFace = getFaceByIndex(referenceCollider, faceIndex);

        // Calculate the face normal in world space
        glm::vec3 transformedNormal = referenceFace.getFaceNormal();
        if (glm::length(transformedNormal) < epsilon) continue; // Skip degenerate faces

        // Calculate the centroid of the reference face (in world coordinates)
        glm::vec3 planePoint = referenceFace.faceVertices[0];

        // Perform Sutherland-Hodgman clipping for each plane
        std::vector<glm::vec3> inputVertices = clippedVertices;
        clippedVertices.clear();

        for (size_t i = 0; i < inputVertices.size(); ++i) {
            glm::vec3 currentVertex = inputVertices[i];
            glm::vec3 nextVertex = inputVertices[(i + 1) % inputVertices.size()];

            // Calculate distances from vertices to the plane, adjusted for numerical stability
            float distCurrent = glm::dot(transformedNormal, currentVertex - planePoint);
            float distNext = glm::dot(transformedNormal, nextVertex - planePoint);
            distCurrent = (glm::abs(distCurrent) < epsilon) ? 0.0f : distCurrent;
            distNext = (glm::abs(distNext) < epsilon) ? 0.0f : distNext;

            // Case 1: Both points are inside (negative or zero distance)
            if (distCurrent <= 0 && distNext <= 0) {
                clippedVertices.push_back(nextVertex);
            }
            // Case 2: Current point is outside, next point is inside
            else if (distCurrent > 0 && distNext <= 0) {
                if (glm::abs(distNext - distCurrent) > epsilon) { // Ensure the division is safe
                    glm::vec3 intersection = currentVertex + (nextVertex - currentVertex) * (-distCurrent / (distNext - distCurrent));
                    clippedVertices.push_back(intersection);
                }
                clippedVertices.push_back(nextVertex);
            }
            // Case 3: Current point is inside, next point is outside
            else if (distCurrent <= 0 && distNext > 0) {
                if (glm::abs(distNext - distCurrent) > epsilon) { // Ensure the division is safe
                    glm::vec3 intersection = currentVertex + (nextVertex - currentVertex) * (-distCurrent / (distNext - distCurrent));
                    clippedVertices.push_back(intersection);
                }
            }
            // Case 4: Both points are outside, do nothing
        }
    }

    // If no vertices remain after clipping, check the edges
    if (clippedVertices.empty()) {

        clippedVertices = HandleEdgeCollision(incidentFace, ReferenceFace);
    }

    // Remove duplicate or nearly overlapping points with precision handling
    if (!clippedVertices.empty()) {

        std::vector<glm::vec3> uniqueVertices;
        for (const auto& vertex : clippedVertices) {
            bool isDuplicate = false;
            for (const auto& uniqueVertex : uniqueVertices) {
                if (glm::length(vertex - uniqueVertex) < epsilon) {
                    isDuplicate = true;
                    break;
                }
            }
            if (!isDuplicate) {
                uniqueVertices.push_back(vertex);
            }
        }

        std::cout << uniqueVertices.size() << std::endl;
        return uniqueVertices;  // Return the cleaned points of contact
    }

    // bruh
    else {
        std::cout << "empty" << std::endl; // ideally, should never happen.
        return clippedVertices;
    }
}

//Expanding Polytope Algorithm
//Find minimum translation vector to resolve collision
constexpr auto EPA_TOLERANCE = 0.0001;
constexpr auto EPA_MAX_NUM_FACES = 64;
constexpr auto EPA_MAX_NUM_LOOSE_EDGES = 32;
constexpr auto EPA_MAX_NUM_ITERATIONS = 64;

// from Kevin Moran's implementation.
gCollisionResult EPA(GJKsimplex& simplex, Polytope* coll1, Polytope* coll2) {
    struct Face {
        glm::vec3 verts[3];  // Triangle vertices
        glm::vec3 normal;    // Face normal
        float distance;      // Distance from origin along the normal
        glm::vec3 support1[3];  // Support points in world space for collider1
        glm::vec3 support2[3];  // Support points in world space for collider2
    };

    //Initialize with final simplex from GJK
    Face faces[EPA_MAX_NUM_FACES]{};

    faces[0] = { {simplex.a, simplex.b, simplex.c}, glm::normalize(glm::cross(simplex.b - simplex.a, simplex.c - simplex.a)), glm::dot(glm::normalize(glm::cross(simplex.b - simplex.a, simplex.c - simplex.a)), simplex.a),
                 {coll1->support(-glm::normalize(glm::cross(simplex.b - simplex.a, simplex.c - simplex.a))), coll1->support(-glm::normalize(glm::cross(simplex.b - simplex.a, simplex.c - simplex.a))), coll1->support(-glm::normalize(glm::cross(simplex.b - simplex.a, simplex.c - simplex.a)))},
                 {coll2->support(glm::normalize(glm::cross(simplex.b - simplex.a, simplex.c - simplex.a))), coll2->support(glm::normalize(glm::cross(simplex.b - simplex.a, simplex.c - simplex.a))), coll2->support(glm::normalize(glm::cross(simplex.b - simplex.a, simplex.c - simplex.a)))} };

    faces[1] = { {simplex.a, simplex.c, simplex.d}, glm::normalize(glm::cross(simplex.c - simplex.a, simplex.d - simplex.a)), glm::dot(glm::normalize(glm::cross(simplex.c - simplex.a,simplex.d - simplex.a)), simplex.a),
                 {coll1->support(-glm::normalize(glm::cross(simplex.c - simplex.a, simplex.d - simplex.a))), coll1->support(-glm::normalize(glm::cross(simplex.c - simplex.a, simplex.d - simplex.a))), coll1->support(-glm::normalize(glm::cross(simplex.c - simplex.a, simplex.d - simplex.a)))},
                 {coll2->support(glm::normalize(glm::cross(simplex.c - simplex.a, simplex.d - simplex.a))), coll2->support(glm::normalize(glm::cross(simplex.c - simplex.a, simplex.d - simplex.a))), coll2->support(glm::normalize(glm::cross(simplex.c - simplex.a, simplex.d - simplex.a)))} };

    faces[2] = { {simplex.a,simplex.d, simplex.b}, glm::normalize(glm::cross(simplex.d - simplex.a, simplex.b - simplex.a)), glm::dot(glm::normalize(glm::cross(simplex.d - simplex.a,simplex.b - simplex.a)), simplex.a),
                 {coll1->support(-glm::normalize(glm::cross(simplex.d - simplex.a, simplex.b - simplex.a))), coll1->support(-glm::normalize(glm::cross(simplex.d - simplex.a, simplex.b - simplex.a))), coll1->support(-glm::normalize(glm::cross(simplex.d - simplex.a, simplex.b - simplex.a)))},
                 {coll2->support(glm::normalize(glm::cross(simplex.d - simplex.a, simplex.b - simplex.a))), coll2->support(glm::normalize(glm::cross(simplex.d - simplex.a, simplex.b - simplex.a))), coll2->support(glm::normalize(glm::cross(simplex.d - simplex.a,simplex.b - simplex.a)))} };

    faces[3] = { {simplex.b, simplex.d, simplex.c}, glm::normalize(glm::cross(simplex.d - simplex.b, simplex.c - simplex.b)), glm::dot(glm::normalize(glm::cross(simplex.d - simplex.b, simplex.c - simplex.b)), simplex.b),
                 {coll1->support(-glm::normalize(glm::cross(simplex.d - simplex.b, simplex.c - simplex.b))), coll1->support(-glm::normalize(glm::cross(simplex.d - simplex.b, simplex.c - simplex.b))), coll1->support(-glm::normalize(glm::cross(simplex.d - simplex.b, simplex.c - simplex.b)))},
                 {coll2->support(glm::normalize(glm::cross(simplex.d - simplex.b, simplex.c - simplex.b))), coll2->support(glm::normalize(glm::cross(simplex.d - simplex.b, simplex.c - simplex.b))), coll2->support(glm::normalize(glm::cross(simplex.d - simplex.b, simplex.c - simplex.b)))} };

    int num_faces = 4;
    int closest_face;

    // EPA main loop
    for (int iterations = 0; iterations < EPA_MAX_NUM_ITERATIONS; iterations++) {
        closest_face = 0;
        float min_dist = faces[0].distance;
        for (int i = 1; i < num_faces; i++) {
            if (faces[i].distance < min_dist) {
                min_dist = faces[i].distance;
                closest_face = i;
            }
        }

        glm::vec3 search_dir = faces[closest_face].normal;
        glm::vec3 p1 = coll1->support(-search_dir);
        glm::vec3 p2 = coll2->support(search_dir);
        glm::vec3 p = p2 - p1;

        // GGs, we have found collision details :D
        if (glm::dot(p, search_dir) - min_dist < EPA_TOLERANCE) {
            gCollisionResult result;
            result.collisionNormal = faces[closest_face].normal;
            result.penetrationDepth = glm::dot(p, search_dir);

            //find the contact manifold here:

            // initialize the storage for reference and incident faces:
            std::tuple<std::vector<int>, glm::vec3> incidentFace;
            std::tuple<std::vector<int>, glm::vec3> referenceFace;

            int refOrInci, referenceFaceIndex, incidentFaceIndex;

            // find the manifold information
            findContactManifoldFaces(coll1, coll2, faces[closest_face].normal, referenceFace, incidentFace, referenceFaceIndex, incidentFaceIndex, refOrInci);

            // set some params
            //result.incidentFace = incidentFace;
            //result.referenceFace = referenceFace;

            if (refOrInci == 1) {
                result.referenceCollider = coll1;
                result.incidentCollider = coll2;
            }
            else if (refOrInci == 2) {
                result.referenceCollider = coll2;
                result.incidentCollider = coll1;
            }

            result.referenceFacce = getFaceByIndex(*result.referenceCollider, referenceFaceIndex);
            result.incidentFacce = getFaceByIndex(*result.incidentCollider, incidentFaceIndex);

            // set the manifold:
            result.manifold = clipFaceAgainstReference(result.incidentFacce, *result.referenceCollider, result.referenceFacce);

            // this result should hopefully contain everything we need to resolve the collision.
            return result;
        }

        // Rebuild the polytope with loose edges, if the algorithm hasn't converged yet.
        glm::vec3 loose_edges[EPA_MAX_NUM_LOOSE_EDGES][2];
        int num_loose_edges = 0;

        for (int i = 0; i < num_faces; i++) {
            if (glm::dot(faces[i].normal, p - faces[i].verts[0]) > 0) {
                for (int j = 0; j < 3; j++) {
                    glm::vec3 current_edge[2] = { faces[i].verts[j], faces[i].verts[(j + 1) % 3] };
                    bool found_edge = false;
                    for (int k = 0; k < num_loose_edges; k++) {
                        if (loose_edges[k][1] == current_edge[0] && loose_edges[k][0] == current_edge[1]) {
                            loose_edges[k][0] = loose_edges[num_loose_edges - 1][0];
                            loose_edges[k][1] = loose_edges[num_loose_edges - 1][1];
                            num_loose_edges--;
                            found_edge = true;
                            break;
                        }
                    }
                    if (!found_edge) {
                        loose_edges[num_loose_edges][0] = current_edge[0];
                        loose_edges[num_loose_edges][1] = current_edge[1];
                        num_loose_edges++;
                    }
                }

                faces[i] = faces[num_faces - 1];
                num_faces--;
                i--;
            }
        }

        for (int i = 0; i < num_loose_edges; i++) {
            if (num_faces >= EPA_MAX_NUM_FACES) break;
            glm::vec3 normal = glm::normalize(glm::cross(loose_edges[i][0] - loose_edges[i][1], loose_edges[i][0] - p));
            faces[num_faces] = { {loose_edges[i][0], loose_edges[i][1], p}, normal, glm::dot(normal, loose_edges[i][0]),
                                 {coll1->support(-normal), coll1->support(-normal), coll1->support(-normal)},
                                 {coll2->support(normal), coll2->support(normal), coll2->support(normal)} };

            if (glm::dot(faces[num_faces].verts[0], faces[num_faces].normal) < 0) {
                std::swap(faces[num_faces].verts[0], faces[num_faces].verts[1]);
                faces[num_faces].normal = -faces[num_faces].normal;
            }
            num_faces++;
        }
    }

    // we are effed.
    printf("EPA did not converge\n");
    return gCollisionResult{
        glm::vec3(0, 0, 0), // default collision normal
        0.0f                 // default penetration depth
    };
}
