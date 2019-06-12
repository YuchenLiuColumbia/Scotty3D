#include <float.h>
#include <assert.h>
#include <string>
#include <stdlib.h>
#include "meshEdit.h"
#include "mutablePriorityQueue.h"
#include "error_dialog.h"

using namespace std;

namespace CMU462 {

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
  // This method should split the given edge and return an iterator to the
  // newly inserted vertex. The halfedge of this vertex should point along
  // the edge that was split, rather than the new edges.
  
  /* A basic pointer reference for *modified surface* is provided below.
     Positions of halfedges could be infered accordingly.
              v2
               o
              /|\
          e1 / | \ e4
            / e01 \
           / f1|f4 \
       v3 o-e03oe04-o v4
           \ f2|f3 /
            \ e02 /
          e2 \ | / e3
              \|/
               o
              v1
  */

  // All new elements needed in the splitted figure.
  VertexIter v0 = newVertex();

  HalfedgeIter h01 = newHalfedge(), 
               h02 = newHalfedge(), 
               h03 = newHalfedge(), 
               h04 = newHalfedge(), 
               h05 = newHalfedge(), 
               h06 = newHalfedge(), 
               h07 = newHalfedge(), 
               h08 = newHalfedge();

  EdgeIter e01 = newEdge(), 
           e02 = newEdge(), 
           e03 = newEdge(), 
           e04 = newEdge();

  FaceIter f1 = newFace(), 
           f2 = newFace(), 
           f3 = newFace(), 
           f4 = newFace();

  // Existing elements, and will be deleted after splitting.
  HalfedgeIter h1 = e0->halfedge(), 
               h2 = h1->twin();

  FaceIter f01 = h1->face(),
           f02 = h2->face();
  // EdgeIter e0 should also be deleted after the operation.
  
  // Existing elements, and will still exist after splitting.
  HalfedgeIter h3 = h1->next(), 
               h4 = h3->next(), 
               h5 = h2->next(), 
               h6 = h5->next();

  VertexIter v1 = h1->vertex(), 
             v2 = h2->vertex(), 
             v3 = h4->vertex(), 
             v4 = h6->vertex();

  EdgeIter e1 = h3->edge(), 
           e2 = h4->edge(), 
           e3 = h5->edge(), 
           e4 = h6->edge();
  
  // Reassign relationships and values of *new pointers*.
  v0->halfedge() = h01;
  v0->position = (v1->position + v2->position) / 2.;

  e01->halfedge() = h01;
  e02->halfedge() = h03;
  e03->halfedge() = h06;
  e04->halfedge() = h08;

  f1->halfedge() = h01;
  f2->halfedge() = h03;
  f3->halfedge() = h04;
  f4->halfedge() = h02;

  h01->setNeighbors(h3, h02, v0, e01, f1);
  h02->setNeighbors(h07, h01, v2, e01, f4);
  h03->setNeighbors(h06, h04, v1, e02, f2);
  h04->setNeighbors(h5, h03, v0, e02, f3);
  h05->setNeighbors(h01, h06, v3, e03, f1);
  h06->setNeighbors(h4, h05, v0, e03, f2);
  h07->setNeighbors(h6, h08, v0, e04, f4);
  h08->setNeighbors(h04, h07, v4, e04, f3);

  // Reassign relationships of *old pointers*.
  h3->face() = f1, h3->next() = h05;
  h4->face() = f2, h4->next() = h03;
  h5->face() = f3, h5->next() = h08;
  h6->face() = f4, h6->next() = h02;

  v1->halfedge() = h5;
  v2->halfedge() = h3;
  v3->halfedge() = h4;
  v4->halfedge() = h6;

  // Delete extra elements.
  deleteFace(f01);
  deleteFace(f02);
  deleteEdge(e0);
  deleteHalfedge(h1);
  deleteHalfedge(h2);

  return v0;
}

VertexIter HalfedgeMesh::collapseEdge(EdgeIter e) {
  // This method should collapse the given edge and return an iterator to
  // the new vertex created by the collapse.

  // Pre-processing
  if (e->isBoundary()) {
    showError("Mesh contain boundaries cannot be collapsed.");
    return e->halfedge()->vertex();
  }

  // Elements to be deleted after collapse.
  HalfedgeIter h0 = e->halfedge(),
               h1 = h0->twin();
  
  VertexIter v01 = h0->vertex(),
             v02 = h1->vertex();
  // EdgeIter e should be deleted also.

  // Elements that not sure to be deleted or not.
  FaceIter f0 = h0->face(),
           f1 = h1->face();

  HalfedgeIter h01 = h0->next(),
               h11 = h1->next(),
               h02 = h01, 
               h12 = h11;

  while (h02->next() != h0) { h02 = h02->next(); }
  while (h12->next() != h1) { h12 = h12->next(); }

  EdgeIter e1 = h02->edge(),
           e2 = h11->edge();
  
  // Elements to be modified, but not deleted.
  HalfedgeIter h03 = h01->twin(),
               h04 = h02->twin(),
               h13 = h11->twin(),
               h14 = h12->twin();

  vector<HalfedgeIter> vertex_change_halfedge; //change ->vertex()
  HalfedgeIter temp = h11;
  while (temp->twin() != h02) {
    temp = temp->twin()->next();
    vertex_change_halfedge.push_back(temp);
  }
  
  temp = h01;
  while (temp->twin() != h12) {
    temp = temp->twin()->next();
    vertex_change_halfedge.push_back(temp);
  }
  

  VertexIter v1 = h01->twin()->vertex(),
             v2 = h02->vertex(),
             v3 = h11->twin()->vertex(),
             v4 = h12->vertex();

  EdgeIter e3 = h01->edge(),
           e4 = h12->edge();
  
  // New vertex created.
  VertexIter v = newVertex();
  
  // This operation should be done before creating something new, to avoid
  // messing up the logic.
  // BOOL VALUES MUST BE SET WHEN DECLARED.
  bool f0_is_tri = false, f1_is_tri = false;
  if (v1 == v2)
    f0_is_tri = true;
  if (v3 == v4)
    f1_is_tri = true;
  
  if (f0_is_tri) {
    if (h0->next()->edge()->isBoundary() || 
      h0->next()->next()->edge()->isBoundary()) {
      showError("Mesh contain boundaries cannot be collapsed.");
      return h0->vertex();
    }
  }

  if (f1_is_tri) {
    if (h1->next()->edge()->isBoundary() || 
      h1->next()->next()->edge()->isBoundary()) {
      showError("Mesh contain boundaries cannot be collapsed.");
      return h1->vertex();
    }
  }

  // This is always true
  v->position = e->centroid();
  v->halfedge() = h03->next();

  // Remapping
  if (f0_is_tri) {
    h03->setNeighbors(h03->next(), h04, v1, e3, h03->face());
    h04->setNeighbors(h04->next(), h03, v, e3, h04->face());
    v1->halfedge() = h03;
  }
  else {
    h01->setNeighbors(h01->next(), h03, v, e3, f0);
    h02->setNeighbors(h01, h04, v2, e1, f0);
    h03->setNeighbors(h03->next(), h01, v1, e3, h03->face());
    h04->setNeighbors(h04->next(), h02, v, e1, h04->face());
    f0->halfedge() = h02;
    e1->halfedge() = h02;
  }
  e3->halfedge() = h03;

  if (f1_is_tri) {
    h13->setNeighbors(h13->next(), h14, v3, e4, h13->face());
    h14->setNeighbors(h14->next(), h13, v, e4, h14->face());
    v3->halfedge() = h13;
  }
  else {
    h11->setNeighbors(h11->next(), h13, v, e2, f1);
    h12->setNeighbors(h11, h14, v4, e4, f1);
    h13->setNeighbors(h13->next(), h11, v3, e2, h13->face());
    h14->setNeighbors(h14->next(), h12, v, e4, h14->face());
    f1->halfedge() = h11;
    e2->halfedge() = h11;
  }
  e4->halfedge() = h14;


  for (int i = 0; i < vertex_change_halfedge.size(); i++) {
    vertex_change_halfedge[i]->vertex() = v;
  }  

  // Delete elements
  if (f0_is_tri) {
    deleteEdge(e1);
    deleteHalfedge(h01);
    deleteHalfedge(h02);
    deleteFace(f0);
  }

  if (f1_is_tri) {
    deleteEdge(e2);
    deleteHalfedge(h11);
    deleteHalfedge(h12);
    deleteFace(f1);
  }

  deleteVertex(v01);
  deleteVertex(v02);
  deleteHalfedge(h0);
  deleteHalfedge(h1);
  deleteEdge(e);

  return v;
}

VertexIter HalfedgeMesh::collapseFace(FaceIter f) {
  // TODO: (meshEdit)
  // This method should collapse the given face and return an iterator to
  // the new vertex created by the collapse.

  // Old Elements to be deleted.
  HalfedgeIter h = f->halfedge(),
               temp = h,
               dest = h;
  vector<HalfedgeIter> halfedge_self_delete;
  vector<HalfedgeIter> halfedge_twin_delete;
  vector<EdgeIter> edge_delete;
  vector<VertexIter> vertex_delete;
  vector<FaceIter> face_delete;

  do {
    halfedge_self_delete.push_back(temp);
    halfedge_twin_delete.push_back(temp->twin());
    edge_delete.push_back(temp->edge());
    vertex_delete.push_back(temp->vertex());
    temp = temp->next();
  } while (temp != h);
  // The (size())th item in the vector is the same with the [0].
  halfedge_self_delete.push_back(temp);
  halfedge_twin_delete.push_back(temp->twin());
  edge_delete.push_back(temp->edge());
  vertex_delete.push_back(temp->vertex());

  // New vertex created in the center of face.
  VertexIter v = newVertex();
  v->position = f->centroid();
  
  // Reassign linking relationships.
  int count = edge_delete.size() - 1;
  while (count > 0) {
    // Deal with triangles
    temp = halfedge_twin_delete[count];
    dest = halfedge_twin_delete[count-1];
    while (temp->next() != dest) {
      temp = temp->next();
      temp->vertex() = v;
      temp->edge()->halfedge() = temp;
      temp->face()->halfedge() = temp;
      temp = temp->twin();
    }
    // Deal with polygon (>3)
    if (dest->next()->next() != temp) {
      temp->next() = dest->next();
      temp = temp->next();
    }
    // Deal with triangle on edge 
    else {
      temp = temp->twin();
      temp->twin() = dest->next()->twin();
      dest->next()->twin()->twin() = temp;
      edge_delete.push_back(temp->edge());
      face_delete.push_back(dest->face());
      halfedge_self_delete.push_back(dest->next());
      halfedge_self_delete.push_back(dest->next()->next());
      temp->edge() = dest->next()->edge();
      temp->twin()->vertex()->halfedge() = temp->twin();
      dest->next() = temp;
    }
    count--;
  }

  v->halfedge() = temp;

  // Delete useless elements.

  for (int i = 1; i < halfedge_self_delete.size(); i++) {
    deleteHalfedge(halfedge_self_delete[i]);
  }
  
  for (int i = 1; i < halfedge_twin_delete.size(); i++) {
    deleteHalfedge(halfedge_twin_delete[i]);
  }

  for (int i = 1; i < edge_delete.size(); i++) {
    deleteEdge(edge_delete[i]);
  }

  for (int i = 1; i < vertex_delete.size(); i++) {
    deleteVertex(vertex_delete[i]);
  }

  for (int i = 0; i < face_delete.size(); i++) {
    deleteFace(face_delete[i]);
  }
  
  deleteFace(f);

  return v;
}

FaceIter HalfedgeMesh::eraseVertex(VertexIter v) {
  // This method should replace the given vertex and all its neighboring
  // edges and faces with a single face, returning the new face.

  // Existing elements, to be deleted.
  vector<EdgeIter> edge_list;
  vector<FaceIter> face_list;
  vector<HalfedgeIter> halfedge_list;
  vector<HalfedgeIter> to_be_deleted;

  // Use an iteration to add all to-be-deleted elements into vector.
  HalfedgeIter head = v->halfedge();
  HalfedgeIter temp = head;
  Vector3D loc = head->face()->normal();
  bool is_even = true;

  do {
    // Check whether the erased faces are even.
    if (is_even && (loc.x - temp->face()->normal().x > 0.01 ||
      loc.y != temp->face()->normal().y > 0.01 || 
      loc.z != temp->face()->normal().z > 0.01)) {
      is_even = false;
    }
    // Iteration.
    halfedge_list.push_back(temp);
    temp = temp->twin();
    halfedge_list.push_back(temp);
    edge_list.push_back(temp->edge());
    face_list.push_back(temp->face());
    temp = temp->next();
  } while (temp != head);

  if (!is_even) {
    showError("Normals not even, might create non-manifold surface.");
  }

  // New elements needed for this feature.
  FaceIter f0 = newFace();

  // Assign new values to halfedges and vertices, and delete useless halfedges.
  head = temp->next();
  HalfedgeIter roam = head;

  while (!halfedge_list.empty()) {
    temp = halfedge_list[halfedge_list.size()-1];
    halfedge_list.pop_back();
    to_be_deleted.push_back(temp);
    roam->vertex()->halfedge() = roam;
    roam->face() = f0;

    while (roam->next() != temp) {
      roam = roam->next();
      roam->face() = f0;
    }

    temp = halfedge_list[halfedge_list.size()-1];
    halfedge_list.pop_back();
    to_be_deleted.push_back(temp);
    roam->next() = temp->next();
    roam = roam->next();
  }

  // Here, head == roam.
  f0->halfedge() = head;

  // Delete useless edges, faces and vertices.
  EdgeIter edge_deletion;
  FaceIter face_deletion;

  while (!edge_list.empty()) {
    edge_deletion = edge_list[edge_list.size()-1];
    edge_list.pop_back();
    deleteEdge(edge_deletion);
  }

  while (!face_list.empty()) {
    face_deletion = face_list[face_list.size()-1];
    face_list.pop_back();
    deleteFace(face_deletion);
  }

  deleteVertex(v);

  while (!to_be_deleted.empty()) {
    temp = to_be_deleted[to_be_deleted.size()-1];
    to_be_deleted.pop_back();
    deleteHalfedge(temp);
  }

  return f0;
}

FaceIter HalfedgeMesh::eraseEdge(EdgeIter e) {
  // This method should erase the given edge and return an iterator to the
  // merged face.

  // Preprocess: check if is boundary.
  if (e->isBoundary()) {
    return e->halfedge()->face();
  }

  // Elements to be deleted.
  HalfedgeIter h1 = e->halfedge(),
               h2 = h1->twin();
  
  FaceIter f1 = h1->face(),
           f2 = h2->face();
  // EdgeIter e should also be deleted.

  // Elements that not sure to be deleted or not.
  VertexIter v1 = h1->vertex(),
             v2 = h2->vertex();

  // One line scenario. Strictly speaking, it should not happen if user is 
  // operating properly. But life is hard, so we should take care of it.
  if (h1->next() == h2 || h2->next() == h1) { 
    if (f1 == f2) {
      HalfedgeIter temp;
      if (v1->degree() == 1) {
        temp = h2->next();
        while (temp->next() != h2)
          temp = temp->next();
        temp->next() = h1->next();
        h2->vertex()->halfedge() = h1->next();
        f1->halfedge() = h1->next();
        deleteEdge(e);
        deleteHalfedge(h1);
        deleteHalfedge(h2);
        deleteVertex(v1);
        return f1;
      }
      else if (v2->degree() == 1) {
        temp = h1->next();
        while (temp->next() != h1)
          temp = temp->next();
        temp->next() = h2->next();
        h1->vertex()->halfedge() = h2->next();
        f1->halfedge() = h2->next();
        deleteEdge(e);
        deleteHalfedge(h1);
        deleteHalfedge(h2);
        deleteVertex(v2);
        return f1;
      }
    }
  }

  // New face to be added. That happens in usual case.
  FaceIter f = newFace();

  // Elements only modified.
  HalfedgeIter h12 = h1->next(),
               h22 = h2->next(),
               h11 = h12,
               h21 = h22;
  
  while (h11->next() != h1) {
    h11->face() = f;
    h11 = h11->next();
  }

  while (h21->next() != h2) {
    h21->face() = f;
    h21 = h21->next();
  }

  // Reassign
  h11->next() = h22;
  h11->face() = f;
  h21->next() = h12;
  h21->face() = f;
  v1->halfedge() = h22;
  v2->halfedge() = h12;
  f->halfedge() = h11;

  // Delete useless elements.
  deleteFace(f1);
  deleteFace(f2);
  deleteHalfedge(h1);
  deleteHalfedge(h2);
  deleteEdge(e);

  return f;
}

EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should flip the given edge and return an iterator to the
  // flipped edge.

  HalfedgeIter h0 = e0->halfedge();
  // Preprocessing
  if (e0->isBoundary() || h0->face()->isBoundary() || 
    h0->twin()->face()->isBoundary()) {
    return e0;
  }

  if (h0->face()->degree() != 3 || h0->twin()->face()->degree() != 3) {
    if (abs(h0->face()->normal().x) - abs(h0->twin()->face()->normal().x) > 0.0001 ||
      abs(h0->face()->normal().y) - abs(h0->twin()->face()->normal().y) > 0.0001 ||
      abs(h0->face()->normal().z) - abs(h0->twin()->face()->normal().z) > 0.0001) {
      showError("Edge cannot be flipped.");
      return e0;
    }
  }

  // Elements to be modified
  // HalfedgeIter h0 has been defined
  HalfedgeIter h1 = h0->twin(),
               h01 = h0->next(),
               h02 = h01->next(),
               h11 = h1->next(),
               h12 = h11->next(),
               h03 = h02,
               h13 = h12;

  while (h03->next() != h0) {
    h03 = h03->next();
  }

  while (h13->next() != h1) {
    h13 = h13->next();
  }

  VertexIter v0 = h0->vertex(),
             v1 = h1->vertex(),
             v2 = h02->vertex(),
             v3 = h12->vertex();
  
  FaceIter f0 = h0->face(),
           f1 = h1->face();

  // Another preprocessing about non-exist face
  if (f0->degree() == 3 && 
    ((v0->position - v2->position).unit() - 
    (v3->position - v0->position).unit()).norm2() < 0.001) {
    showError("Edge cannot be flipped.");
    return e0;
  }
  if (f1->degree() == 3 &&
    ((v3->position - v1->position).unit() - 
    (v1->position - v2->position).unit()).norm2() < 0.001) {
    showError("Edge cannot be flipped.");
    return e0;
  }
  
  // Reassign
  v0->halfedge() = h11;
  v1->halfedge() = h01;
  v2->halfedge() = h02;
  v3->halfedge() = h12;

  h0->setNeighbors(h02, h1, v3, e0, f0);
  h1->setNeighbors(h12, h0, v2, e0, f1);
  h01->setNeighbors(h1, h01->twin(), v1, h01->edge(), f1);
  h03->next() = h11;
  h11->setNeighbors(h0, h11->twin(), v0, h11->edge(), f0);
  h13->next() = h01;

  f0->halfedge() = h0;
  f1->halfedge() = h1;

  return e0;
}

void HalfedgeMesh::subdivideQuad(bool useCatmullClark) {
  // Step I: Compute the vertex positions for the subdivided mesh.
  for (auto e = edges.begin(); e != edges.end(); e++) {
    if (e->isBoundary() && useCatmullClark) {
      showError("Catmull-clark can't be used in meshes with boundaries.");
      return;
    }
  }
  if (useCatmullClark) {
    computeCatmullClarkPositions();
  } else {
    computeLinearSubdivisionPositions();
  }

  // Step II: Assign a unique index (starting at 0) to each vertex, edge,
  // andface in the original mesh.
  assignSubdivisionIndices();

  // Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
  // the element indices defined above. 
  vector<vector<Index> > subDFaces;
  vector<Vector3D> subDVertices;
  buildSubdivisionFaceList(subDFaces);
  buildSubdivisionVertexList(subDVertices);

  // Step IV: Pass the list of vertices and quads to a routine that clears
  // the internal data for this halfedge mesh, and builds new halfedge.
  rebuild(subDFaces, subDVertices);
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * simple linear interpolation, e.g., the edge midpoints and face
 * centroids.
 */
void HalfedgeMesh::computeLinearSubdivisionPositions() {
  for (auto v = vertices.begin(); v != vertices.end(); v++) {
    v->newPosition = v->position;
  }

  for (auto e = edges.begin(); e != edges.end(); e++) {
    e->newPosition = e->centroid();
  }

  for (auto f = faces.begin(); f != faces.end(); f++) {
    f->newPosition = f->centroid();
  }

  return;
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * the Catmull-Clark rules for subdivision.
 */
void HalfedgeMesh::computeCatmullClarkPositions() {
  for (auto f = faces.begin(); f != faces.end(); f++) {
    f->newPosition = f->centroid();
  }

  for (auto e = edges.begin(); e != edges.end(); e++) {
    e->newPosition = (e->halfedge()->face()->newPosition + 
                      e->halfedge()->twin()->face()->newPosition + 
                      e->centroid() + e->centroid()) / 4.;
  }

  for (auto v = vertices.begin(); v != vertices.end(); v++) {
    HalfedgeIter temp = v->halfedge(), 
                 end = temp;
    int count = 0;
    Vector3D Q(0.), R(0.), S(0.);

    do {
      Q += temp->face()->newPosition;
      count ++;
      temp = temp->twin()->next();
    } while (temp != end);
    Q /= count;
    count = 0;

    do {
      R += temp->edge()->centroid();
      count ++;
      temp = temp->twin()->next();
    } while (temp != end);
    R /= count;

    S = v->position;

    v->newPosition = (Q + R * 2. + S * (v->degree() - 3.)) / v->degree();
  }
  return;
}

/**
 * Assign a unique integer index to each vertex, edge, and face in
 * the mesh, starting at 0 and incrementing by 1 for each element.
 * These indices will be used as the vertex indices for a mesh
 * subdivided using Catmull-Clark (or linear) subdivision.
 */
void HalfedgeMesh::assignSubdivisionIndices() {
  Index count = 0;

  for (auto v = vertices.begin(); v != vertices.end(); v++) {
    v->index = count++;
  }

  for (auto e = edges.begin(); e != edges.end(); e++) {
    e->index = count++;
  }

  for (auto f = faces.begin(); f != faces.end(); f++) {
    f->index = count++;
  }

  return;
}

/**
 * Build a flat list containing all the vertex positions for a
 * Catmull-Clark (or linear) subdivison of this mesh.  The order of
 * vertex positions in this list must be identical to the order
 * of indices assigned to Vertex::newPosition, Edge::newPosition,
 * and Face::newPosition.
 */
void HalfedgeMesh::buildSubdivisionVertexList(vector<Vector3D>& subDVertices) {
  // Resize the vertex list so that it can hold all the vertices.
  subDVertices.resize(vertices.size() + edges.size() + faces.size());
  // Iterate and place positions in the index places.
  for (auto v = vertices.begin(); v != vertices.end(); v++) {
    subDVertices[v->index] = v->newPosition;
  }

  for (auto e = edges.begin(); e != edges.end(); e++) {
    subDVertices[e->index] = e->newPosition;
  }

  for (auto f = faces.begin(); f != faces.end(); f++) {
    subDVertices[f->index] = f->newPosition;
  }
  return;
}

/**
 * Build a flat list containing all the quads in a Catmull-Clark
 * (or linear) subdivision of this mesh.  Each quad is specified
 * by a vector of four indices (i,j,k,l), which come from the
 * members Vertex::index, Edge::index, and Face::index.  Note that
 * the ordering of these indices is important because it determines
 * the orientation of the new quads; it is also important to avoid
 * "bowties."  For instance, (l,k,j,i) has the opposite orientation
 * of (i,j,k,l), and if (i,j,k,l) is a proper quad, then (i,k,j,l)
 * will look like a bowtie.
 */
void HalfedgeMesh::buildSubdivisionFaceList(vector<vector<Index> >& subDFaces) {
  HalfedgeIter hf, h0;

  for (auto f = faces.begin(); f != faces.end(); f++) {
    hf = f->halfedge();
    h0 = hf;
    do {
      vector<Index> temp;
      temp.push_back(h0->edge()->index);
      temp.push_back(h0->next()->vertex()->index);
      temp.push_back(h0->next()->edge()->index);
      temp.push_back(f->index);
      
      subDFaces.push_back(temp);
      h0 = h0->next();
    } while (h0 != hf);
  }

  return;
}

FaceIter HalfedgeMesh::bevelVertex(VertexIter v) {
  // TODO This method should replace the vertex v with a face, corresponding to
  // a bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelVertexComputeNewPositions (which you also have to
  // implement!)

  showError("bevelVertex() not implemented.");
  return facesBegin();
}

FaceIter HalfedgeMesh::bevelEdge(EdgeIter e) {
  // TODO This method should replace the edge e with a face, corresponding to a
  // bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelEdgeComputeNewPositions (which you also have to
  // implement!)

  showError("bevelEdge() not implemented.");
  return facesBegin();
}

FaceIter HalfedgeMesh::bevelFace(FaceIter f) {
  // TODO This method should replace the face f with an additional, inset face
  // (and ring of faces around it), corresponding to a bevel operation. It
  // should return the new face.  NOTE: This method is responsible for updating
  // the *connectivity* of the mesh only---it does not need to update the vertex
  // positions.  These positions will be updated in
  // HalfedgeMesh::bevelFaceComputeNewPositions (which you also have to
  // implement!)

  showError("bevelFace() not implemented.");
  return facesBegin();
}


void HalfedgeMesh::bevelFaceComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double normalShift,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled face.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the originalVertexPositions array) to compute an offset vertex
  // position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); hs++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //

}

void HalfedgeMesh::bevelVertexComputeNewPositions(
    Vector3D originalVertexPosition, vector<HalfedgeIter>& newHalfedges,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled vertex.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., hs.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.

}

void HalfedgeMesh::bevelEdgeComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled edge.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); i++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //

}

void HalfedgeMesh::splitPolygons(vector<FaceIter>& fcs) {
  for (auto f : fcs) splitPolygon(f);
}

void HalfedgeMesh::splitPolygon(FaceIter f) {
  // TODO: (meshedit) 
  // Triangulate a polygonal face
  if (f->degree() == 3) {
    return;
  }
  // New Elements
  FaceIter f1 = newFace();
  EdgeIter e1 = newEdge();
  HalfedgeIter h0 = newHalfedge(),
               h1 = newHalfedge();
  
  // Old Elements to be changed
  HalfedgeIter h00 = f->halfedge(),
               h01 = h00->next(),
               h02 = h01->next(),
               h03 = h02;
  
  while (h03->next() != h00) {
    h03 = h03->next();
  }
  
  // Reassign
  f1->halfedge() = h1;
  e1->halfedge() = h0;
  h01->next() = h0;
  h01->face() = f;
  h0->setNeighbors(h00, h1, h02->vertex(), e1, f);
  h1->setNeighbors(h02, h0, h00->vertex(), e1, f1);
  h03->next() = h1;
  while (h02 != h1) {
    h02->face() = f1;
    h02 = h02->next();
  }

  splitPolygon(f1);
  return;
}

EdgeRecord::EdgeRecord(EdgeIter& _edge) : edge(_edge) {
  // TODO: (meshEdit)
  // Compute the combined quadric from the edge endpoints.
  // -> Build the 3x3 linear system whose solution minimizes the quadric error
  //    associated with these two endpoints.
  // -> Use this system to solve for the optimal position, and store it in
  //    EdgeRecord::optimalPoint.
  // -> Also store the cost associated with collapsing this edg in
  //    EdgeRecord::Cost.
}

void MeshResampler::upsample(HalfedgeMesh& mesh)
{
  // Step 1: Mark old vertices as 'old' ones.
  // Step 2: Calculate new positions of old vertices.
  for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
    v->isNew = false;
    Vector3D sum(0.);
    HalfedgeIter temp = v->halfedge(),
                 end = temp;
    int degree = 0;
    float u = 0.;
    do {
      sum += temp->twin()->vertex()->position;
      degree ++;
      temp = temp->twin()->next();
    } while (temp != end);

    if (degree == 3) {
      u = 3. / 16;
    }
    else {
      u = 3. / (8 * degree);
    }

    v->newPosition = (1. - degree * u) * v->position + u * sum;
  }
  cout << "2" << endl;
  
  // Step 3: Calculate positions of new vertices (in the edge).
  for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    HalfedgeIter h = e->halfedge();
    e->newPosition = 0.375 * h->vertex()->position + 
                     0.375 * h->twin()->vertex()->position +
                     0.125 * h->next()->next()->vertex()->position +
                     0.125 * h->twin()->next()->next()->vertex()->position;
    cout << e->newPosition << endl;
  }
  cout << "3" << endl;
  // Step 4: Create New Edges using SplitEdge()
  int count = mesh.nEdges();
  EdgeIter e = mesh.edgesBegin();
  for (int i = 0; i < count; i++) {
    // Get the next edge in a temp iterator
    EdgeIter next_edge = e;
    next_edge ++;
    
    // Split the previous edge
    Vector3D this_pos = e->newPosition;
    VertexIter this_vertex = mesh.splitEdge(e);
    this_vertex->isNew = true;
    this_vertex->newPosition = this_pos;
    this_vertex->halfedge()->edge()->isNew = false;
    this_vertex->halfedge()->twin()->next()->edge()->isNew = true;
    this_vertex->halfedge()->twin()->next()->twin()->next()
               ->edge()->isNew = false;
    this_vertex->halfedge()->twin()->next()->twin()->next()
               ->twin()->next()->edge()->isNew = true;

    // Go to next edge
    e = next_edge;
  } 
  cout << "4" << endl;
  
  // Step 5: Flip edges with connection between new and old vertices.
  for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    if (e->isNew && e->halfedge()->vertex()->isNew != 
      e->halfedge()->twin()->vertex()->isNew) {
      mesh.flipEdge(e);
    }
  }
  
  cout << "5" << endl;
  // Step 6: Reassign the vertex positions
  for (auto v = mesh.verticesBegin(); v != mesh. verticesEnd(); v++) {
    v->position = v->newPosition;
  }
  cout << "6" << endl;
  
  return;
}

void MeshResampler::downsample(HalfedgeMesh& mesh) {
  // TODO: (meshEdit)
  // Compute initial quadrics for each face by simply writing the plane equation
  // for the face in homogeneous coordinates. These quadrics should be stored
  // in Face::quadric
  // -> Compute an initial quadric for each vertex as the sum of the quadrics
  //    associated with the incident faces, storing it in Vertex::quadric
  // -> Build a priority queue of edges according to their quadric error cost,
  //    i.e., by building an EdgeRecord for each edge and sticking it in the
  //    queue.
  // -> Until we reach the target edge budget, collapse the best edge. Remember
  //    to remove from the queue any edge that touches the collapsing edge
  //    BEFORE it gets collapsed, and add back into the queue any edge touching
  //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
  //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
  //    top of the queue.
  showError("downsample() not implemented.");
}

void MeshResampler::resample(HalfedgeMesh& mesh) {
  // TODO: (meshEdit)
  // Compute the mean edge length.
  // Repeat the four main steps for 5 or 6 iterations
  // -> Split edges much longer than the target length (being careful about
  //    how the loop is written!)
  // -> Collapse edges much shorter than the target length.  Here we need to
  //    be EXTRA careful about advancing the loop, because many edges may have
  //    been destroyed by a collapse (which ones?)
  // -> Now flip each edge if it improves vertex degree
  // -> Finally, apply some tangential smoothing to the vertex positions
  showError("resample() not implemented.");
}

}  // namespace CMU462
