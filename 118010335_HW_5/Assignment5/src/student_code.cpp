#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Task 1.
    std::vector<Vector2D> next_points;
    for (int i = 0; i < points.size() - 1; ++i)
    {
      Vector2D interpolated_point = (1.0 - t) * points[i] + t * points[i + 1];
      next_points.push_back(interpolated_point);
    }
    return next_points;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Task 2.
    std::vector<Vector3D> next_points;
    for (int i = 0; i < points.size() - 1; ++i)
    {
      Vector3D interpolated_point = (1.0 - t) * points[i] + t * points[i + 1];
      next_points.push_back(interpolated_point);
    }
    return next_points;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Task 2.
    std::vector<Vector3D> buffer = points;
    for (int i = 0; i < points.size() - 1; ++i)
    {
      buffer = evaluateStep(buffer, t);
    }
    return buffer[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Task 2.
    std::vector<Vector3D> column_control_points; 
    // evaluate each row
    for (int i = 0; i < controlPoints.size(); ++i)
    {
      column_control_points.push_back(evaluate1D(controlPoints[i], u));
    }
    // evaluate the column
    return evaluate1D(column_control_points, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Task 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    Vector3D normal_sum(0, 0, 0);
    // Get a halfedge from the vertex
    HalfedgeCIter current_halfedge = halfedge();
    HalfedgeCIter start_halfedge = current_halfedge;

    do
    {
      // When a -> b -> c is counterclockwise
      // cross(b-a,c-a) is pointing outwards
      Vector3D a = position;
      Vector3D b = current_halfedge->next()->vertex()->position;
      Vector3D c = current_halfedge->next()->next()->vertex()->position;
      // Area = cross(b-a,c-a).norm()
      // Unit normal vector = cross(b-a,c-a) / cross(b-a,c-a).norm()
      // Weighted normal vector = Area * Unit normal vector = cross(b-a,c-a)
      normal_sum += cross(b-a,c-a);
      current_halfedge = current_halfedge->twin()->next();
    } 
    while (current_halfedge != start_halfedge);

    return normal_sum.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Task 4.
    // This method should flip the given edge and return an iterator to the flipped edge.

    // If either neighbouring face of e0 is on a boundary loop, return
    if (e0->halfedge()->isBoundary() || e0->halfedge()->twin()->isBoundary())
    {
      return e0;
    }

    // Record all the halfedges of the two faces associated with e0
    // Face a
    HalfedgeIter ha0 = e0->halfedge();
    HalfedgeIter ha1 = ha0->next();
    HalfedgeIter ha2 = ha1->next();
    // Face b
    HalfedgeIter hb0 = ha0->twin();
    HalfedgeIter hb1 = hb0->next();
    HalfedgeIter hb2 = hb1->next();
    
    // Record the two faces associated with e0
    // Face a
    FaceIter fa = ha0->face();
    // Face b
    FaceIter fb = hb0->face();   

    // Record all the vertices of the two faces associated with e0
    // Shared vertices
    VertexIter v0 = ha0->vertex();
    VertexIter v1 = hb0->vertex();
    // Face a
    VertexIter va0 = ha2->vertex();
    // Face b
    VertexIter vb0 = hb2->vertex();
    
    // Modify the halfedges
    ha0->setNeighbors(hb2, hb0, va0, e0, fa);
    hb0->setNeighbors(ha2, ha0, vb0, e0, fb);
    hb2->setNeighbors(ha1, hb2->twin(), vb0, hb2->edge(), fa);
    ha1->setNeighbors(ha0, ha1->twin(), v1, ha1->edge(), fa);
    ha2->setNeighbors(hb1, ha2->twin(), va0, ha2->edge(), fb);
    hb1->setNeighbors(hb0, hb1->twin(), v0, hb1->edge(), fb);

    // Modify the vertices
    v0->halfedge() = hb1;
    v1->halfedge() = ha1;

    // Modify the faces
    fa->halfedge() = ha0;
    fb->halfedge() = hb0;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Task 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    // Record all the halfedges of the two faces associated with e0
    // Face a
    HalfedgeIter ha0 = e0->halfedge();
    HalfedgeIter ha1 = ha0->next();
    HalfedgeIter ha2 = ha1->next();
    // Face b
    HalfedgeIter hb0 = ha0->twin();
    HalfedgeIter hb1 = hb0->next();
    HalfedgeIter hb2 = hb1->next();
    
    // Record the two faces associated with e0
    // Face a
    FaceIter fa = ha0->face();
    // Face b
    FaceIter fb = hb0->face();   

    // Record all the vertices of the two faces associated with e0
    // Shared vertices
    VertexIter v0 = ha0->vertex();
    VertexIter v1 = hb0->vertex();
    // Face a
    VertexIter va0 = ha2->vertex();
    // Face b
    VertexIter vb0 = hb2->vertex();

    /* Boundary edge handling */
    // Face b is a virtual boundary face
    // Swap a and b
    // So that face a is a virtual boundary face
    // i.e. fa->isBoundary() will become true
    if (fb->isBoundary())
    {
      // Face a
      hb0 = e0->halfedge();
      hb1 = hb0->next();
      hb2 = hb1->next();
      // Face b
      ha0 = hb0->twin();
      ha1 = ha0->next();
      ha2 = ha1->next();
      
      // Record the two faces associated with e0
      // Face a
      fa = ha0->face();
      // Face b
      fb = hb0->face();   

      // Record all the vertices of the two faces associated with e0
      // Shared vertices
      v0 = ha0->vertex();
      v1 = hb0->vertex();
      // Face a
      va0 = ha2->vertex();
      // Face b
      vb0 = hb2->vertex();
    }
    // Face a is a virtual boundary face
    if (fa->isBoundary())
    {
      // Create new elements
      // The middle point
      VertexIter m = newVertex();
      // The new halfedges on the original e0
      HalfedgeIter ha0_twin = newHalfedge();
      HalfedgeIter hb0_twin = newHalfedge();
      // The new halfedges from the middle point to other vertices
      HalfedgeIter hmb0 = newHalfedge();
      // The new halfedges from other vertices to the middle point
      HalfedgeIter hb0m = newHalfedge();
      // The other half of e0
      EdgeIter e0_spilt = newEdge();
      // The new edges from the middle point to other vertices
      EdgeIter emb0 = newEdge();
      // The face of hmb0 -> hb2 -> hb0
      FaceIter fb0 = newFace();

      m->position = (v0->position + v1->position)/2.0;
      m->halfedge() = ha0_twin;

      // Face fa
      ha0->setNeighbors(hb0_twin, ha0_twin, v0, e0, fa);
      hb0_twin->setNeighbors(ha1, hb0, m, e0_spilt, fa);
      // ha1->setNeighbors(ha2, ha1->twin(), v1, ha1->edge(), fa);
      // ha2->setNeighbors(ha0, ha2->twin(), va0, ha2->edge(), fa);

      // Face fb
      hmb0->setNeighbors(hb2, hb0m, m, emb0, fb);
      hb2->setNeighbors(hb0, hb2->twin(), vb0, hb2->edge(), fb);
      hb0->setNeighbors(hmb0, hb0_twin, v1, e0_spilt, fb);
      
      // Face fb0
      hb0m->setNeighbors(ha0_twin, hmb0, vb0, emb0, fb0);
      ha0_twin->setNeighbors(hb1, ha0, m, e0, fb0);
      hb1->setNeighbors(hb0m, hb1->twin(), v0, hb1->edge(), fb0);
      
      // Set the halfedge of each face
      fa->halfedge() = ha0;
      fb->halfedge() = hmb0;
      fb0->halfedge() = hb0m;

      // Set the halfedge of each edge
      e0->halfedge() = ha0;
      e0_spilt->halfedge() = hb0;
      emb0->halfedge() = hmb0;

      // e0 and e0_split do not cut cross a triangle
      e0->isNew = false;
      e0_spilt->isNew = false;

      // emb0 cuts cross a triangle
      emb0->isNew = true;

      // m is new vertex
      m->isNew = true;

      return m;
    }
    /* Boundary edge handling */

    // Create new elements
    // The middle point
    VertexIter m = newVertex();
    // The new halfedges on the original e0
    HalfedgeIter ha0_twin = newHalfedge();
    HalfedgeIter hb0_twin = newHalfedge();
    // The new halfedges from the middle point to other vertices
    HalfedgeIter ha0m = newHalfedge();
    HalfedgeIter hmb0 = newHalfedge();
    // The new halfedges from other vertices to the middle point
    HalfedgeIter hma0 = newHalfedge();
    HalfedgeIter hb0m = newHalfedge();
    // The other half of e0
    EdgeIter e0_spilt = newEdge();
    // The new edges from the middle point to other vertices
    EdgeIter ema0 = newEdge();
    EdgeIter emb0 = newEdge();
    // The face of hmb0 -> hb2 -> hb0
    FaceIter fb0 = newFace();
    // The face of hb0m -> ha0_twin -> hb1
    FaceIter fb1 = newFace();

    m->position = (v0->position + v1->position)/2.0;
    m->halfedge() = ha0_twin;

    // Face fa
    hma0->setNeighbors(ha2, ha0m, m, ema0, fa);
    ha0->setNeighbors(hma0, ha0_twin, v0, e0, fa);
    ha2->setNeighbors(ha0, ha2->twin(), va0, ha2->edge(), fa);

    // Face fb
    ha0m->setNeighbors(hb0_twin, hma0, va0, ema0, fb);
    hb0_twin->setNeighbors(ha1, hb0, m, e0_spilt, fb);
    ha1->setNeighbors(ha0m, ha1->twin(), v1, ha1->edge(), fb);

    // Face fb0
    hmb0->setNeighbors(hb2, hb0m, m, emb0, fb0);
    hb2->setNeighbors(hb0, hb2->twin(), vb0, hb2->edge(), fb0);
    hb0->setNeighbors(hmb0, hb0_twin, v1, e0_spilt, fb0);
    
    // Face fb1
    hb0m->setNeighbors(ha0_twin, hmb0, vb0, emb0, fb1);
    ha0_twin->setNeighbors(hb1, ha0, m, e0, fb1);
    hb1->setNeighbors(hb0m, hb1->twin(), v0, hb1->edge(), fb1);
    
    // Set the halfedge of each face
    fa->halfedge() = hma0;
    fb->halfedge() = ha0m;
    fb0->halfedge() = hmb0;
    fb1->halfedge() = hb0m;

    // Set the halfedge of each edge
    e0->halfedge() = ha0;
    e0_spilt->halfedge() = hb0;
    ema0->halfedge() = hma0;
    emb0->halfedge() = hmb0;

    // e0 and e0_split do not cut cross a triangle
    e0->isNew = false;
    e0_spilt->isNew = false;

    // ema0 and emb0 cut cross a triangle
    ema0->isNew = true;
    emb0->isNew = true;

    // m is new vertex
    m->isNew = true;

    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Task 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) 
    {
      // degree n
      size_t n = v->degree();
      // constant u
      double u = 0.0;
      if (n == 3)
      {
        u = 3.0 / 16.0;
      }
      else
      {
        u = 3.0 / (8.0 * (double) n);
      }
      
      Vector3D neighbours(0, 0, 0);
      HalfedgeCIter h_start = v->halfedge();
      HalfedgeCIter h = h_start;
      // Interior
      if (!v->isBoundary())
      {
        do
        {
          // h->twin() points to v
          // h->twin()->vertex() is the neighbour
          neighbours += h->twin()->vertex()->position;
          // h->twin()->next() points from v
          h = h->twin()->next();
        } 
        while (h != h_start);
        v->newPosition = (1.0 - (double) n * u) * v->position + u * neighbours;
      }
      // Boundary
      else
      {
        int count = 0;
        do
        {
          if (h->twin()->isBoundary() || h->isBoundary())
          {
            // h->twin() points to v
            // h->twin()->vertex() is the neighbour
            neighbours += h->twin()->vertex()->position;
            // h->twin()->next() points from v
            count++;
          }
          h = h->twin()->next();
        } 
        while (h != h_start);
        v->newPosition = (3.0 / 4.0) * v->position + (1.0 / 8.0) * neighbours;
      }
      v->isNew = false;
    }

    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) 
    {
      HalfedgeIter h;
      if (!e->halfedge()->isBoundary())
      {
        h = e->halfedge();
      }
      else if (!e->halfedge()->twin()->isBoundary())
      {
        h = e->halfedge()->twin();
      }
      else
      {
        continue;
      }
      // The two vertices on the edge
      Vector3D v0 = h->vertex()->position;
      Vector3D v1 = h->twin()->vertex()->position;

      // Interior
      if (!h->next()->next()->isBoundary() && !h->twin()->next()->next()->isBoundary())
      {
        // The two vertices around the edge
        Vector3D va0 = h->next()->next()->vertex()->position;
        Vector3D vb0 = h->twin()->next()->next()->vertex()->position;
        e->newPosition = 3.0 / 8.0 * (v0 + v1) + 1.0 / 8.0 * (va0 + vb0);
      }
      // Boundary
      else
      {
        e->newPosition = 1.0 / 2.0 * (v0 + v1);
      }
      e->isNew = false;
    }

    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

    int num_old_edges = (int) mesh.nEdges();
    auto e = mesh.edgesBegin();
    for (int splitted_count = 0; splitted_count < num_old_edges; splitted_count++)
    {
      VertexIter v = mesh.splitEdge(e);
      // Update the position of the new vertex to the position stored in the edge
      v->position = e->newPosition;
      e++;
    }

    // 4. Flip any new edge that connects an old and new vertex.

    for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) 
    {
      if (e->isNew) 
      {
        HalfedgeIter h = e->halfedge();
        VertexIter v0 = h->vertex();
        VertexIter v1 = h->twin()->vertex();
        if ( (v0->isNew && !v1->isNew) || (!v0->isNew && v1->isNew) ) 
        {
          e = mesh.flipEdge(e);
        }
      }
    }

    // 5. Copy the new vertex positions into final Vertex::position.

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) 
    {
      // Consider only the old vertices
      // Since the new vertices have already been updated
      if (v->isNew == false) 
      {
        v->position = v->newPosition;
      }
    }
  }
}
