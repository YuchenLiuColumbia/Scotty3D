#include <float.h>
#include <assert.h>
#include <string>
#include "meshEdit.h"
#include "mutablePriorityQueue.h"
#include "error_dialog.h"

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

  while (h02->next() != h0) {
    h02 = h02->next();
  }

  while (h12->next() != h1) {
    h12 = h12->next();
  }

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
  
  // This operation should be done before creating something new, to avoid
  // messing up the logic.
  bool f0_is_tri, f1_is_tri;
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

  // New vertex created.
  VertexIter v = newVertex();
  v->position = (v01->position + v02->position) / 2.;

  // Remapping

  // This is always true
  v->halfedge() = h04;

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
    f0->halfedge() = h01;
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
  //std::cout << halfedge_self_delete.size() << ' ' << halfedge_twin_delete.size() << ' ' << edge_delete.size() << ' ' << vertex_delete.size() << ' ' << face_delete.size() << std::endl;

  // The (size())th item in the vector is the same with the [0].
  halfedge_self_delete.push_back(temp);
  halfedge_twin_delete.push_back(temp->twin());
  edge_delete.push_back(temp->edge());
  vertex_delete.push_back(temp->vertex());

  //std::cout << halfedge_self_delete.size() << ' ' << halfedge_twin_delete.size() << ' ' << edge_delete.size() << ' ' << vertex_delete.size() << ' ' << face_delete.size() << std::endl;

  // New vertex created in the center of face.
  VertexIter v = newVertex();
  v->position = f->centroid();
  
  // Reassign linking relationships.
  int count = edge_delete.size() - 1;
  while (count > 0) {
    // Deal with triangles
    temp = halfedge_twin_delete[count];
    dest = halfedge_twin_delete[count-1];
    //std::cout << "temp: " << temp->getInfo()[2] << "temp-next: " << temp->getInfo()[4] << "temp-next-twin: " << temp->next()->getInfo()[3] << "dest" << dest->getInfo()[2] << std::endl;
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
        //std::cout << halfedge_self_delete.size() << ' ' << halfedge_twin_delete.size() << ' ' << edge_delete.size() << ' ' << vertex_delete.size() << ' ' << face_delete.size() << std::endl;
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
      //std::cout << halfedge_self_delete.size() << ' ' << halfedge_twin_delete.size() << ' ' << edge_delete.size() << ' ' << vertex_delete.size() << ' ' << face_delete.size() << std::endl;
      //std::cout << "line left: " << temp->getInfo()[2] << "line right: " << temp->twin()->getInfo()[2] << "dest: " << dest->getInfo()[2] << "dest-next:" << dest->getInfo()[4] << std::endl;
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

  showError("flipEdge() not implemented.");
  return EdgeIter();
}

void HalfedgeMesh::subdivideQuad(bool useCatmullClark) {
  // Unlike the local mesh operations (like bevel or edge flip), we will perform
  // subdivision by splitting *all* faces into quads "simultaneously."  Rather
  // than operating directly on the halfedge data structure (which as you've
  // seen
  // is quite difficult to maintain!) we are going to do something a bit nicer:
  //
  //    1. Create a raw list of vertex positions and faces (rather than a full-
  //       blown halfedge mesh).
  //
  //    2. Build a new halfedge mesh from these lists, replacing the old one.
  //
  // Sometimes rebuilding a data structure from scratch is simpler (and even
  // more
  // efficient) than incrementally modifying the existing one.  These steps are
  // detailed below.

  // TODO Step I: Compute the vertex positions for the subdivided mesh.  Here
  // we're
  // going to do something a little bit strange: since we will have one vertex
  // in
  // the subdivided mesh for each vertex, edge, and face in the original mesh,
  // we
  // can nicely store the new vertex *positions* as attributes on vertices,
  // edges,
  // and faces of the original mesh.  These positions can then be conveniently
  // copied into the new, subdivided mesh.
  // [See subroutines for actual "TODO"s]
  if (useCatmullClark) {
    computeCatmullClarkPositions();
  } else {
    computeLinearSubdivisionPositions();
  }

  // TODO Step II: Assign a unique index (starting at 0) to each vertex, edge,
  // and
  // face in the original mesh.  These indices will be the indices of the
  // vertices
  // in the new (subdivided mesh).  They do not have to be assigned in any
  // particular
  // order, so long as no index is shared by more than one mesh element, and the
  // total number of indices is equal to V+E+F, i.e., the total number of
  // vertices
  // plus edges plus faces in the original mesh.  Basically we just need a
  // one-to-one
  // mapping between original mesh elements and subdivided mesh vertices.
  // [See subroutine for actual "TODO"s]
  assignSubdivisionIndices();

  // TODO Step III: Build a list of quads in the new (subdivided) mesh, as
  // tuples of
  // the element indices defined above.  In other words, each new quad should be
  // of
  // the form (i,j,k,l), where i,j,k and l are four of the indices stored on our
  // original mesh elements.  Note that it is essential to get the orientation
  // right
  // here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces should
  // circulate in the same direction as old faces (think about the right-hand
  // rule).
  // [See subroutines for actual "TODO"s]
  vector<vector<Index> > subDFaces;
  vector<Vector3D> subDVertices;
  buildSubdivisionFaceList(subDFaces);
  buildSubdivisionVertexList(subDVertices);

  // TODO Step IV: Pass the list of vertices and quads to a routine that clears
  // the
  // internal data for this halfedge mesh, and builds new halfedge data from
  // scratch,
  // using the two lists.
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
  // TODO For each vertex, assign Vertex::newPosition to
  // its original position, Vertex::position.

  // TODO For each edge, assign the midpoint of the two original
  // positions to Edge::newPosition.

  // TODO For each face, assign the centroid (i.e., arithmetic mean)
  // of the original vertex positions to Face::newPosition.  Note
  // that in general, NOT all faces will be triangles!
  showError("computeLinearSubdivisionPositions() not implemented.");
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
  // TODO The implementation for this routine should be
  // a lot like HalfedgeMesh::computeLinearSubdivisionPositions(),
  // except that the calculation of the positions themsevles is
  // slightly more involved, using the Catmull-Clark subdivision
  // rules. (These rules are outlined in the Developer Manual.)

  // TODO face

  // TODO edges

  // TODO vertices
  showError("computeCatmullClarkPositions() not implemented.");
}

/**
 * Assign a unique integer index to each vertex, edge, and face in
 * the mesh, starting at 0 and incrementing by 1 for each element.
 * These indices will be used as the vertex indices for a mesh
 * subdivided using Catmull-Clark (or linear) subdivision.
 */
void HalfedgeMesh::assignSubdivisionIndices() {
  // TODO Start a counter at zero; if you like, you can use the
  // "Index" type (defined in halfedgeMesh.h)

  // TODO Iterate over vertices, assigning values to Vertex::index

  // TODO Iterate over edges, assigning values to Edge::index

  // TODO Iterate over faces, assigning values to Face::index
  showError("assignSubdivisionIndices() not implemented.");
}

/**
 * Build a flat list containing all the vertex positions for a
 * Catmull-Clark (or linear) subdivison of this mesh.  The order of
 * vertex positions in this list must be identical to the order
 * of indices assigned to Vertex::newPosition, Edge::newPosition,
 * and Face::newPosition.
 */
void HalfedgeMesh::buildSubdivisionVertexList(vector<Vector3D>& subDVertices) {
  // TODO Resize the vertex list so that it can hold all the vertices.

  // TODO Iterate over vertices, assigning Vertex::newPosition to the
  // appropriate location in the new vertex list.

  // TODO Iterate over edges, assigning Edge::newPosition to the appropriate
  // location in the new vertex list.

  // TODO Iterate over faces, assigning Face::newPosition to the appropriate
  // location in the new vertex list.
  showError("buildSubdivisionVertexList() not implemented.");
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
  // TODO This routine is perhaps the most tricky step in the construction of
  // a subdivision mesh (second, perhaps, to computing the actual Catmull-Clark
  // vertex positions).  Basically what you want to do is iterate over faces,
  // then for each for each face, append N quads to the list (where N is the
  // degree of the face).  For this routine, it may be more convenient to simply
  // append quads to the end of the list (rather than allocating it ahead of
  // time), though YMMV.  You can of course iterate around a face by starting
  // with its first halfedge and following the "next" pointer until you get
  // back to the beginning.  The tricky part is making sure you grab the right
  // indices in the right order---remember that there are indices on vertices,
  // edges, AND faces of the original mesh.  All of these should get used.  Also
  // remember that you must have FOUR indices per face, since you are making a
  // QUAD mesh!

  // TODO iterate over faces
  // TODO loop around face
  // TODO build lists of four indices for each sub-quad
  // TODO append each list of four indices to face list
  showError("buildSubdivisionFaceList() not implemented.");
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
  showError("splitPolygon() not implemented.");
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
// This routine should increase the number of triangles in the mesh using Loop
// subdivision.
{
  // TODO: (meshEdit)
  // Compute new positions for all the vertices in the input mesh, using
  // the Loop subdivision rule, and store them in Vertex::newPosition.
  // -> At this point, we also want to mark each vertex as being a vertex of the
  //    original mesh.
  // -> Next, compute the updated vertex positions associated with edges, and
  //    store it in Edge::newPosition.
  // -> Next, we're going to split every edge in the mesh, in any order.  For
  //    future reference, we're also going to store some information about which
  //    subdivided edges come from splitting an edge in the original mesh, and
  //    which edges are new, by setting the flat Edge::isNew. Note that in this
  //    loop, we only want to iterate over edges of the original mesh.
  //    Otherwise, we'll end up splitting edges that we just split (and the
  //    loop will never end!)
  // -> Now flip any new edge that connects an old and new vertex.
  // -> Finally, copy the new vertex positions into final Vertex::position.

  // Each vertex and edge of the original surface can be associated with a
  // vertex in the new (subdivided) surface.
  // Therefore, our strategy for computing the subdivided vertex locations is to
  // *first* compute the new positions
  // using the connectity of the original (coarse) mesh; navigating this mesh
  // will be much easier than navigating
  // the new subdivided (fine) mesh, which has more elements to traverse.  We
  // will then assign vertex positions in
  // the new mesh based on the values we computed for the original mesh.

  // Compute updated positions for all the vertices in the original mesh, using
  // the Loop subdivision rule.

  // Next, compute the updated vertex positions associated with edges.

  // Next, we're going to split every edge in the mesh, in any order.  For
  // future
  // reference, we're also going to store some information about which
  // subdivided
  // edges come from splitting an edge in the original mesh, and which edges are
  // new.
  // In this loop, we only want to iterate over edges of the original
  // mesh---otherwise,
  // we'll end up splitting edges that we just split (and the loop will never
  // end!)

  // Finally, flip any new edge that connects an old and new vertex.

  // Copy the updated vertex positions to the subdivided mesh.
  showError("upsample() not implemented.");
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
