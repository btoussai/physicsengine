package cataclysm.wrappers;

import java.util.Iterator;

import math.vector.Vector3f;

/**
 * Repr�sente une face d'une enveloppe convexe pour les collision.
 * 
 * @author Briac
 *
 */
public class ConvexHullWrapperFace implements Iterable<ConvexHullWrapperHalfEdge> {

	private static class FaceIterator implements Iterator<ConvexHullWrapperHalfEdge> {
		private ConvexHullWrapperHalfEdge e0;
		private ConvexHullWrapperHalfEdge current;
		boolean first_loop = true;

		FaceIterator(ConvexHullWrapperHalfEdge e0) {
			this.current = e0;
			this.e0 = e0;
		}

		@Override
		public boolean hasNext() {
			return current != e0 || first_loop;
		}

		@Override
		public ConvexHullWrapperHalfEdge next() {
			first_loop = false;
			current = current.getNext();
			return current;
		}

	}

	ConvexHullWrapperData data;
	int index;
	int edge0;

	public ConvexHullWrapperFace() {
	}

	public ConvexHullWrapperFace(ConvexHullWrapperFace face, ConvexHullWrapperData data) {
		this.data = data;
		this.edge0 = face.edge0;
		this.index = face.index;
	}

	public Vector3f getNormal() {
		return data.faceNormals[index];
	}

	public Vector3f getCentroid() {
		return data.faceCentroids[index];
	}

	public float getPlaneOffset() {
		return data.planeOffsets[index];
	}

	public float signedDistance(Vector3f position) {
		return Vector3f.dot(position, getNormal()) - getPlaneOffset();
	}

	public ConvexHullWrapperHalfEdge getEdge0() {
		return data.edges[edge0];
	}

	public void setEdge0(int edge0) {
		this.edge0 = edge0;
	}

	public void setIndex(int index) {
		this.index = index;
	}

	public void projectOnPlane(Vector3f vertex) {
		Vector3f normal = getNormal();
		float projection = Vector3f.dot(normal, vertex) - getPlaneOffset();
		vertex.x -= projection * normal.x;
		vertex.y -= projection * normal.y;
		vertex.z -= projection * normal.z;
	}

	@Override
	public Iterator<ConvexHullWrapperHalfEdge> iterator() {
		return new FaceIterator(getEdge0());
	}

}
