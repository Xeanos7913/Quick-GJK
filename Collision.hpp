#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <memory>
#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <glm/gtx/norm.hpp>
#include "VkCalcium.hpp"

struct Collider {
    glm::vec3 pos;            // Origin in world space
    glm::mat3 matRS;          // Rotation/scale component of model matrix
    glm::mat3 matRS_inverse;
    Entity* owner;       // you need to plug the actual owner object here, of your own engine.
    virtual glm::vec3 support(glm::vec3 dir) = 0;
};

// Helper hash function for std::pair<int, int> to be used in unordered_map
struct PairHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        auto hash1 = std::hash<int>{}(p.first);
        auto hash2 = std::hash<int>{}(p.second);
        return hash1 ^ (hash2 << 1); // Combine hashes
    }
};

struct PolyPlane {
    glm::vec3 normal;
    glm::vec3 point;
    glm::vec3 point1;
};

struct Polytope : Collider {
    float* points;  // (x0 y0 z0 x1 y1 z1 etc)
    int num_points;

    // Each face now has a vector of vertex indices and a normal (vec3)
    std::vector<std::tuple<std::vector<int>, glm::vec3>> faces;
    // Plane representations of each face
    std::vector<PolyPlane> planes;
    
    // Adjacency list: Index corresponds to the face index, value is the list of neighboring planes
    std::vector<std::vector<PolyPlane>> plane_neighbors;

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

    // Helper to convert a local PolyPlane to World Space
    PolyPlane toWorldSpace(const PolyPlane& localPlane) const {
        PolyPlane worldPlane;
        
        // Get the inverse transpose of the model matrix for correct normal transformation
        worldPlane.normal = matRS_inverse * localPlane.normal;
        
        // Transform the point on the plane into world space
        worldPlane.point = matRS * localPlane.point;
        
        return worldPlane;
    }

    // Robust lookup using the face index directly (Transforms the result to World Space)
    std::vector<PolyPlane> getPlaneNeighborsByIndex(int faceIndex) const {
        std::vector<PolyPlane> worldNeighbors;
        if (faceIndex >= 0 && faceIndex < plane_neighbors.size()) {
            worldNeighbors.reserve(plane_neighbors[faceIndex].size());
            for (const auto& localNeighbor : plane_neighbors[faceIndex]) {
                worldNeighbors.push_back((localNeighbor));
            }
        }
        return worldNeighbors;
    }
    
    // Optional: Get a specific face plane in world space
    PolyPlane getPlaneByIndex(int faceIndex) const {
        if (faceIndex >= 0 && faceIndex < planes.size()) {
            return (planes[faceIndex]);
        }
        return PolyPlane{glm::vec3(0), glm::vec3(0)};
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

struct PolyEdge {
    int a, b;

    bool operator==(const PolyEdge& other) const {
        return a == other.a && b == other.b;
    }
};

void addOrCancelEdge(std::vector<PolyEdge>& edges, int a, int b)
{
    for (auto it = edges.begin(); it != edges.end(); ++it) {
        if (it->a == b && it->b == a) {
            edges.erase(it); // internal edge cancels
            return;
        }
    }
    edges.push_back({a, b});
}

// A half-open face during construction.
// We use indices into the original point cloud throughout.
struct HullFace {
    int v[3];           // Vertex indices (CCW winding when viewed from outside)
    glm::vec3 normal;   // Outward unit normal
    glm::vec3 centroid; // Average of v[0..2], used for plane equation
 
    // Signed distance from this face's plane to point p.
    // Positive = outside (in front of face), negative = inside.
    float distanceTo(const glm::vec3& p) const {
        return glm::dot(normal, p - centroid);
    }
};
 
// An oriented edge: directed from a to b.
// The horizon is a loop of directed edges such that each edge's reverse
// (b, a) is present among the surviving (non-visible) faces.
struct Edge {
    int a, b;
    bool operator==(const Edge& o) const { return a == o.a && b == o.b; }
};
 
struct EdgeHash {
    size_t operator()(const Edge& e) const {
        return std::hash<int>{}(e.a) ^ (std::hash<int>{}(e.b) << 16);
    }
};
 
// Compute outward face normal given three points in CCW order.
// CCW winding convention: normal points toward the reader when
// the triangle is drawn counter-clockwise.
static glm::vec3 faceNormal(const glm::vec3& a,
                             const glm::vec3& b,
                             const glm::vec3& c) {
    return glm::normalize(glm::cross(b - a, c - a));
}
 
// Build a HullFace from three vertex indices.
// 'interior' is any point known to be strictly inside the hull (the centroid
// of the 4 seed points works).  We use it to orient the normal outward.
static HullFace makeFace(int i0, int i1, int i2,
                          const std::vector<glm::vec3>& pts,
                          const glm::vec3& interior) {
    HullFace f;
    f.v[0] = i0; f.v[1] = i1; f.v[2] = i2;
    f.centroid = (pts[i0] + pts[i1] + pts[i2]) / 3.0f;
    f.normal = faceNormal(pts[i0], pts[i1], pts[i2]);
 
    // Flip if the normal points toward the interior instead of away.
    if (glm::dot(f.normal, interior - f.centroid) > 0.0f) {
        std::swap(f.v[1], f.v[2]);
        f.normal = -f.normal;
    }
    return f;
}

// Points closer than this to a plane are considered "on" the plane (coplanar).
// Raise it if you get degenerate/duplicate faces on near-flat geometry.
static constexpr float HULL_EPSILON = 1e-5f;

std::shared_ptr<Polytope> buildConvexHull(const std::vector<glm::vec3>& points) {
 
    const int n = static_cast<int>(points.size());
    if (n < 4) return nullptr;
 
    // ─── Step 1: Find seed tetrahedron ──────────────────────────────────────
    //
    // We need 4 non-coplanar points to start.  We pick them by finding the
    // 6 axis-aligned extrema first (±x, ±y, ±z), then choosing the pair
    // with the greatest mutual distance, then the point furthest from the
    // line between them, then the point furthest from the plane of those 3.
    //
    // This is more robust than just taking indices 0-3 from the input.
 
    // Find extremal point indices along each axis
    std::array<int,6> extrema = {0,0,0,0,0,0};
    for (int i = 1; i < n; ++i) {
        if (points[i].x < points[extrema[0]].x) extrema[0] = i;
        if (points[i].x > points[extrema[1]].x) extrema[1] = i;
        if (points[i].y < points[extrema[2]].y) extrema[2] = i;
        if (points[i].y > points[extrema[3]].y) extrema[3] = i;
        if (points[i].z < points[extrema[4]].z) extrema[4] = i;
        if (points[i].z > points[extrema[5]].z) extrema[5] = i;
    }
 
    // Pick the pair of extrema that are furthest apart (seed edge)
    int s0 = 0, s1 = 1;
    float best = -1.0f;
    for (int a = 0; a < 6; ++a) {
        for (int b = a + 1; b < 6; ++b) {
            float d = glm::distance(points[extrema[a]], points[extrema[b]]);
            if (d > best) { best = d; s0 = extrema[a]; s1 = extrema[b]; }
        }
    }
 
    // Seed triangle: find point furthest from line s0–s1
    glm::vec3 lineDir = glm::normalize(points[s1] - points[s0]);
    int s2 = -1;
    float bestDist = -1.0f;
    for (int i = 0; i < n; ++i) {
        if (i == s0 || i == s1) continue;
        glm::vec3 diff = points[i] - points[s0];
        float d = glm::length(diff - glm::dot(diff, lineDir) * lineDir);
        if (d > bestDist) { bestDist = d; s2 = i; }
    }
    if (s2 == -1 || bestDist < HULL_EPSILON) return nullptr; // All collinear
 
    // Seed tetrahedron: find point furthest from plane s0–s1–s2
    glm::vec3 seedNormal = faceNormal(points[s0], points[s1], points[s2]);
    int s3 = -1;
    float bestPlaneDist = -1.0f;
    for (int i = 0; i < n; ++i) {
        if (i == s0 || i == s1 || i == s2) continue;
        float d = std::abs(glm::dot(seedNormal, points[i] - points[s0]));
        if (d > bestPlaneDist) { bestPlaneDist = d; s3 = i; }
    }
    if (s3 == -1 || bestPlaneDist < HULL_EPSILON) return nullptr; // All coplanar
 
    // Interior reference point: average of the 4 seed vertices.
    // Used to consistently orient face normals outward.
    glm::vec3 interior = (points[s0] + points[s1] + points[s2] + points[s3]) * 0.25f;
 
    // Build the 4 faces of the seed tetrahedron.
    // Each face's normal is oriented away from 'interior'.
    std::vector<HullFace> faces;
    faces.reserve(256);
    faces.push_back(makeFace(s0, s1, s2, points, interior));
    faces.push_back(makeFace(s0, s1, s3, points, interior));
    faces.push_back(makeFace(s0, s2, s3, points, interior));
    faces.push_back(makeFace(s1, s2, s3, points, interior));
 
    // ─── Step 2: Assign outside points to each face ─────────────────────────
    //
    // For each face we maintain a list of points that are "outside" it
    // (positive signed distance > HULL_EPSILON).  A point is assigned to
    // at most one face (the one it is furthest outside of).
    //
    // This partitioning means we never test the full point cloud against
    // every face — it's the key to Quickhull's efficiency.
 
    // outside_sets[i] = indices of points outside faces[i]
    std::vector<std::vector<int>> outside_sets(faces.size());
 
    auto assignPoint = [&](int pi) {
        float maxD = HULL_EPSILON;
        int bestFace = -1;
        for (int fi = 0; fi < (int)faces.size(); ++fi) {
            float d = faces[fi].distanceTo(points[pi]);
            if (d > maxD) { maxD = d; bestFace = fi; }
        }
        if (bestFace >= 0) outside_sets[bestFace].push_back(pi);
    };
 
    for (int i = 0; i < n; ++i) {
        if (i == s0 || i == s1 || i == s2 || i == s3) continue;
        assignPoint(i);
    }
 
    // ─── Step 3: Expand the hull ────────────────────────────────────────────
    //
    // Process each face that has outside points.
    // We use a simple work-queue pattern: faces added later also get processed.
 
    int fi = 0; // current face index
    while (fi < (int)faces.size()) {
        if (outside_sets[fi].empty()) { ++fi; continue; }
 
        // 3a. Find the apex: the outside point furthest from faces[fi].
        //     This point will become a new hull vertex.
        int apex = outside_sets[fi][0];
        float apexDist = faces[fi].distanceTo(points[apex]);
        for (int pi : outside_sets[fi]) {
            float d = faces[fi].distanceTo(points[apex]);
            if (d > apexDist) { apexDist = d; apex = pi; }
        }
 
        // 3b. Find all faces visible from apex.
        //     A face is visible if the apex is outside its plane.
        //     We do a BFS/flood-fill from faces[fi] through face adjacency.
        //
        //     BFS is more robust than testing all faces because floating-point
        //     rounding can make a face appear "inside" even when it's part of
        //     a visible patch.  Adjacency ensures we don't miss connected
        //     visible faces.
 
        // Build edge→face adjacency on the fly for the current hull.
        // edge_to_face[(a,b)] = face index that has directed edge a→b.
        std::unordered_map<Edge, int, EdgeHash> edge_to_face;
        for (int k = 0; k < (int)faces.size(); ++k) {
            const HullFace& f = faces[k];
            edge_to_face[{f.v[0], f.v[1]}] = k;
            edge_to_face[{f.v[1], f.v[2]}] = k;
            edge_to_face[{f.v[2], f.v[0]}] = k;
        }
 
        // BFS flood-fill from fi
        std::unordered_set<int> visible;
        std::vector<int> bfs = {fi};
        visible.insert(fi);
        while (!bfs.empty()) {
            int cur = bfs.back(); bfs.pop_back();
            // Check the 3 neighboring faces (the ones sharing an edge)
            for (int e = 0; e < 3; ++e) {
                int a = faces[cur].v[e];
                int b = faces[cur].v[(e+1)%3];
                // The neighboring face owns the reverse edge (b→a)
                auto it = edge_to_face.find({b, a});
                if (it == edge_to_face.end()) continue;
                int neighbor = it->second;
                if (visible.count(neighbor)) continue;
                if (faces[neighbor].distanceTo(points[apex]) > HULL_EPSILON) {
                    visible.insert(neighbor);
                    bfs.push_back(neighbor);
                }
            }
        }
 
        // 3c. Find the horizon: directed edges on the boundary between
        //     visible and non-visible faces.
        //
        //     An edge (a→b) is on the horizon if:
        //       - Its owning face is visible
        //       - The face owning the reverse edge (b→a) is NOT visible
        //
        //     The horizon edges form a closed loop (for convex hulls).
        //     We collect them as directed edges pointing INTO the visible region
        //     (so the new triangles will have correct winding).
 
        std::vector<Edge> horizon;
        for (int vi : visible) {
            for (int e = 0; e < 3; ++e) {
                int a = faces[vi].v[e];
                int b = faces[vi].v[(e+1)%3];
                auto it = edge_to_face.find({b, a});
                // If the reverse edge has no owner or its owner is not visible,
                // this edge is on the horizon.
                if (it == edge_to_face.end() || !visible.count(it->second)) {
                    horizon.push_back({a, b});
                }
            }
        }
 
        // 3d. Collect outside points from all visible faces.
        //     They will be re-partitioned among the new faces.
        std::vector<int> orphans;
        for (int vi : visible) {
            for (int pi : outside_sets[vi]) {
                if (pi != apex) orphans.push_back(pi);
            }
        }
 
        // 3e. Remove visible faces (mark as deleted, then erase).
        //     We also remove their outside sets at the same time.
        {
            // Sort descending so erasing by index doesn't shift earlier indices.
            std::vector<int> vis_sorted(visible.begin(), visible.end());
            std::sort(vis_sorted.rbegin(), vis_sorted.rend());
            for (int vi : vis_sorted) {
                faces.erase(faces.begin() + vi);
                outside_sets.erase(outside_sets.begin() + vi);
            }
            // fi may have shifted; reset and re-scan from the beginning.
            // This is the simple/correct approach; a production implementation
            // would use a free-list instead of erasing.
            fi = 0;
        }
 
        // 3f. Create new triangular faces: apex + each horizon edge.
        //     The horizon edge direction already gives us the correct winding:
        //     if (a→b) is on the horizon, (apex, a, b) is CCW from outside.
        //
        //     Why? The horizon edge (a→b) is owned by a visible face, meaning
        //     the face's outward normal pointed toward apex.  The non-visible
        //     neighbour owns (b→a), so from outside the hull b→a is CCW on
        //     that face.  The new face (apex, a, b) continues the same
        //     consistent CCW convention.
 
        int new_face_start = static_cast<int>(faces.size());
        for (const Edge& e : horizon) {
            faces.push_back(makeFace(apex, e.a, e.b, points, interior));
            outside_sets.push_back({});
        }
 
        // 3g. Re-assign orphaned outside points to the new faces.
        for (int pi : orphans) {
            float maxD = HULL_EPSILON;
            int bestFace = -1;
            for (int k = new_face_start; k < (int)faces.size(); ++k) {
                float d = faces[k].distanceTo(points[pi]);
                if (d > maxD) { maxD = d; bestFace = k; }
            }
            if (bestFace >= 0) outside_sets[bestFace].push_back(pi);
        }
 
        // fi was reset to 0 after the erase step above, so the while loop
        // will naturally continue from the beginning.
    }
 
    // ─── Step 4: Merge coplanar triangles into n-gons ───────────────────────
    //
    // Quickhull always produces triangles. For a cube that means 12 faces,
    // but a cube only has 6 logical faces. Duplicate normals break the
    // reference-face lookup in clipping-based collision detection, because
    // the lookup finds the face whose normal best matches the collision normal —
    // if two triangles share a normal you get an arbitrary winner, and its
    // neighbor list is incomplete (3 edges instead of 4).
    //
    // Solution: group triangles by coplanarity, then for each group extract
    // the boundary polygon via half-edge cancellation.
    //
    // Half-edge cancellation:
    //   Every directed edge (a→b) inside a coplanar group has a twin (b→a)
    //   belonging to the adjacent triangle in the same group. Collect all
    //   directed edges from every triangle in the group; cancel any pair
    //   (a→b) + (b→a). What remains are the boundary edges — they form a
    //   single closed loop for a convex planar region.
    //
    // Loop reconstruction:
    //   Build a map next_vertex[a] = b for each boundary edge (a→b).
    //   Walk from an arbitrary start vertex, following next_vertex, until
    //   we return to the start. That ordered walk is the polygon.
 
    // 4a. Group triangle indices by coplanar normal.
    //     We use a tolerance-based normal comparison rather than exact equality
    //     because floating-point normals from adjacent triangles can diverge
    //     by small amounts even when geometrically coplanar.
    //
    //     Strategy: for each triangle, search existing groups for a compatible
    //     normal (dot product > 1 - NORMAL_MERGE_DOT_THRESHOLD and same plane
    //     offset). If found, add to that group; otherwise start a new group.
    //     This is O(F²) in group count but F is small for typical convex hulls.
 
    static constexpr float NORMAL_MERGE_DOT_THRESHOLD = 1e-4f; // cos(angle) must exceed 1 - this
 
    struct FaceGroup {
        glm::vec3 normal;       // Representative normal for this group
        float     plane_d;      // Plane offset: dot(normal, any_point_on_plane)
        std::vector<int> tri_indices; // Which triangles (into 'faces') belong here
    };
    std::vector<FaceGroup> groups;
 
    for (int ti = 0; ti < (int)faces.size(); ++ti) {
        const HullFace& hf = faces[ti];
        float d = glm::dot(hf.normal, points[hf.v[0]]);
 
        bool placed = false;
        for (FaceGroup& g : groups) {
            // Check both normal alignment and same plane (via plane offset d).
            bool same_normal = (1.0f - glm::dot(g.normal, hf.normal)) < NORMAL_MERGE_DOT_THRESHOLD;
            bool same_plane  = std::abs(g.plane_d - d) < HULL_EPSILON * 100.0f;
            if (same_normal && same_plane) {
                g.tri_indices.push_back(ti);
                placed = true;
                break;
            }
        }
        if (!placed) {
            FaceGroup g;
            g.normal = hf.normal;
            g.plane_d = d;
            g.tri_indices.push_back(ti);
            groups.push_back(std::move(g));
        }
    }
 
    // 4b. For each group, reconstruct the boundary polygon.
    //
    //     The result is an ordered list of vertex indices forming the polygon
    //     boundary in CCW order (same winding as the source triangles).
 
    struct MergedFace {
        std::vector<int> verts; // Ordered boundary vertices (CCW from outside)
        glm::vec3 normal;
    };
    std::vector<MergedFace> merged_faces;
    merged_faces.reserve(groups.size());
 
    for (const FaceGroup& g : groups) {
        // Collect all directed edges from triangles in this group.
        // Use a map: edge (a,b) -> count of times it appears directed a→b.
        // An internal edge appears once as (a,b) and once as (b,a).
        // A boundary edge appears only as (a,b).
        std::unordered_map<Edge, int, EdgeHash> edge_count;
        for (int ti : g.tri_indices) {
            const HullFace& hf = faces[ti];
            for (int e = 0; e < 3; ++e) {
                Edge fwd = {hf.v[e], hf.v[(e+1)%3]};
                edge_count[fwd]++;
            }
        }
 
        // Boundary edges are those with no reverse twin, i.e. edge (a,b) is
        // boundary iff edge (b,a) does not appear in edge_count.
        // Build next_vertex map for boundary loop walk.
        std::unordered_map<int,int> next_vertex; // next_vertex[a] = b for boundary edge a→b
        for (const auto& [e, cnt] : edge_count) {
            Edge rev = {e.b, e.a};
            if (edge_count.find(rev) == edge_count.end()) {
                // (e.a → e.b) is a boundary edge
                next_vertex[e.a] = e.b;
            }
        }
 
        if (next_vertex.empty()) continue; // Degenerate group, skip
 
        // Walk the boundary loop from an arbitrary start vertex.
        MergedFace mf;
        mf.normal = g.normal;
        int start = next_vertex.begin()->first;
        int cur   = start;
        do {
            mf.verts.push_back(cur);
            auto it = next_vertex.find(cur);
            if (it == next_vertex.end()) break; // Should not happen on valid hull
            cur = it->second;
        } while (cur != start);
 
        merged_faces.push_back(std::move(mf));
    }
 
    // ─── Step 5: Package into Polytope ──────────────────────────────────────
 
    auto polytope = std::make_shared<Polytope>();
 
    // 5a. Copy point data (full original cloud; support() iterates all points)
    polytope->num_points = n;
    polytope->points = new float[n * 3];
    for (int i = 0; i < n; ++i) {
        polytope->points[i*3+0] = points[i].x;
        polytope->points[i*3+1] = points[i].y;
        polytope->points[i*3+2] = points[i].z;
    }
 
    // 5b. Store merged n-gon faces.
    //     Each face is now one polygon per planar region.
    //     A cube produces exactly 6 faces, each with 4 vertices.
    polytope->faces.clear();
    polytope->faces.reserve(merged_faces.size());
    for (const MergedFace& mf : merged_faces) {
        polytope->faces.emplace_back(mf.verts, mf.normal);
    }
 
    // 5c. Build planes[].
    //     point  = first vertex of the polygon (a point on the plane).
    //     point1 = second vertex (for normal recalculation via cross product).
    polytope->planes.clear();
    polytope->planes.reserve(merged_faces.size());
    for (const MergedFace& mf : merged_faces) {
        PolyPlane p;
        p.normal = mf.normal;
        p.point  = points[mf.verts[0]];
        p.point1 = points[mf.verts[1]];
        polytope->planes.push_back(p);
    }
 
    // 5d. Build plane_neighbors[] adjacency list on the merged faces.
    //
    //     Two merged faces are neighbors iff they share at least one edge.
    //     We identify shared edges using the same undirected (min,max) key.
    //     A boundary edge of merged face i that is the boundary of merged face j
    //     means i and j are adjacent — exactly what SAT side-plane clipping needs.
    //
    //     Note: after merging, every edge in merged_faces is a TRUE geometric
    //     boundary (the internal edges were cancelled in Step 4). So adjacency
    //     here reflects actual face-face adjacency on the convex hull.
 
    std::unordered_map<std::pair<int,int>, std::vector<int>, PairHash> edge_to_merged;
    for (int i = 0; i < (int)polytope->faces.size(); ++i) {
        const auto& verts = std::get<0>(polytope->faces[i]);
        for (int j = 0; j < (int)verts.size(); ++j) {
            int a = verts[j];
            int b = verts[(j+1) % verts.size()];
            if (a > b) std::swap(a, b);
            edge_to_merged[{a, b}].push_back(i);
        }
    }
 
    polytope->plane_neighbors.resize(polytope->faces.size());
    for (const auto& [edge, faceList] : edge_to_merged) {
        for (size_t i = 0; i < faceList.size(); ++i) {
            for (size_t j = i + 1; j < faceList.size(); ++j) {
                int f1 = faceList[i];
                int f2 = faceList[j];
                polytope->plane_neighbors[f1].push_back(polytope->planes[f2]);
                polytope->plane_neighbors[f2].push_back(polytope->planes[f1]);
            }
        }
    }
 
    return polytope;
}

void updatePolytope(Polytope& tope) {
    tope.matRS = tope.owner->transform.model;
    tope.matRS_inverse = inverse(tope.matRS);
    tope.pos = tope.owner->transform.getPosition();
}

// Function to extract the vertices of a face by index, transforming them into world coordinates
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
    glm::mat4 modelMatrix = polytope.owner->transform.model;

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
        float scaleMagnitude1 = glm::length(collider1->owner->transform.getScale());
        float scaleMagnitude2 = glm::length(collider2->owner->transform.getScale());

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

struct Face {
    glm::vec3 point;
    glm::vec3 normal;
};

std::array<Face, 6> faces = {
    Face{
        glm::vec3(0.0f, 0.0f, 0.0f), // Front (-Z)
        glm::vec3(0.0f, 0.0f, -1.0f)
    },
    Face{
        glm::vec3(0.0f, 0.0f, 1.0f), // Back (+Z)
        glm::vec3(0.0f, 0.0f, 1.0f)
    },
    Face{
        glm::vec3(0.0f, 0.0f, 0.0f), // Bottom (-Y)
        glm::vec3(0.0f, -1.0f, 0.0f)
    },
    Face{
        glm::vec3(0.0f, 1.0f, 0.0f), // Top (+Y)
        glm::vec3(0.0f, 1.0f, 0.0f)
    },
    Face{
        glm::vec3(0.0f, 0.0f, 0.0f), // Left (-X)
        glm::vec3(-1.0f, 0.0f, 0.0f)
    },
    Face{
        glm::vec3(1.0f, 0.0f, 0.0f), // Right (+X)
        glm::vec3(1.0f, 0.0f, 0.0f)
    }
};

// Function to clip a polygonal face (incidentFace) against the planes (polygonal faces) of a reference collider
std::vector<glm::vec3> clipFaceAgainstReference(const PolygonalFace& incidentFace,
                                                const Polytope& referenceCollider,
                                                const int referenceFace)
{
    std::vector<glm::vec3> clippedVertices = incidentFace.faceVertices;
    const float epsilon = 0.001f;

    for (int i = 0; i < faces.size(); i++) {

        auto face = faces[i];

        glm::vec3 n = glm::transpose(glm::inverse(glm::mat3(referenceCollider.owner->transform.model))) * face.normal;

        if (glm::length(n) < epsilon) continue;

        glm::vec3 P0 = glm::vec3(referenceCollider.owner->transform.model * glm::vec4(face.point, 1.0f));

        std::vector<glm::vec3> inputVertices = clippedVertices;
        clippedVertices.clear();

        for (size_t i = 0; i < inputVertices.size(); ++i) {
            const glm::vec3& A = inputVertices[i];
            const glm::vec3& B = inputVertices[(i + 1) % inputVertices.size()];

            float dA = glm::dot(n, A - P0);
            float dB = glm::dot(n, B - P0);

            // Snap near-zero distances to zero for numerical stability
            if (glm::abs(dA) < epsilon) dA = 0.0f;
            if (glm::abs(dB) < epsilon) dB = 0.0f;

            bool AInside = dA < -epsilon;
            bool BInside = dB < -epsilon;

            bool AOn = glm::abs(dA) <= epsilon;
            bool BOn = glm::abs(dB) <= epsilon;

            // Case 1: both inside
            if ((AInside || AOn) && (BInside || BOn)) {
                clippedVertices.push_back(B);
            }

            // Case 2: A outside, B on plane
            else if (!AInside && !AOn && BOn) {
                clippedVertices.push_back(B);
            }

            // Case 3: A on plane, B outside
            else if (AOn && !BInside && !BOn) {
                continue;
            }

            // Case 4: A inside, B outside
            else if ((AInside || AOn) && (!BInside && !BOn)) {
                float denom = dA - dB;
                if (glm::abs(denom) > epsilon) {
                    float t = dA / denom;
                    clippedVertices.push_back(A + t * (B - A));
                }
            }

            // Case 5: A outside, B inside
            else if ((!AInside && !AOn) && (BInside || BOn)) {
                float denom = dA - dB;
                if (glm::abs(denom) > epsilon) {
                    float t = dA / denom;
                    clippedVertices.push_back(A + t * (B - A));
                }
                clippedVertices.push_back(B);
            }

            // Case 6: both outside -> emit nothing
        }
    }

    return clippedVertices;
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
            result.manifold = clipFaceAgainstReference(result.incidentFacce, *result.referenceCollider, referenceFaceIndex);

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
