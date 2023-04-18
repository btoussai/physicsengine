package cataclysm.wrappers;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import cataclysm.wrappers.ConvexHullWrapperData.EdgeLayout;
import cataclysm.wrappers.ConvexHullWrapperData.FaceLayout;
import cataclysm.wrappers.ConvexHullWrapperData.FloatLayout;
import cataclysm.wrappers.ConvexHullWrapperData.IntLayout;
import math.vector.Matrix3f;
import math.vector.Vector3f;

public class ConvexHullWrapperData {
	
	public static class WrapperModelData {
		public int[] indices;
		public float[] vertices;
	}
	
	public final int faceCount;
	public final int edgeCount;
	public final int vertexCount;
	public final short[] intData;
	public final float[] floatData;
	public final float maxRadius;

	public final Vector3f centerOfMass_hollow = new Vector3f();
	public final Matrix3f inertia_hollow = new Matrix3f();
	public final MassProperties mass_hollow = new MassProperties(0, 0, true, 1.0f);

	public final Vector3f centerOfMass_full = new Vector3f();
	public final Matrix3f inertia_full = new Matrix3f();
	public final MassProperties mass_full = new MassProperties(0, 0, false, 1.0f);
	
	
	public ConvexHullWrapperData(int faceCount, int edgeCount, int vertexCount, short[] intData,
			float[] floatData, float maxRadius, PolyhedralMassProperties poly) {
		this.faceCount = faceCount;
		this.edgeCount = edgeCount;
		this.vertexCount = vertexCount;
		this.intData = intData;
		this.floatData = floatData;
		this.maxRadius = maxRadius;
		poly.computeProperties(this, mass_hollow, centerOfMass_hollow, inertia_hollow);
		poly.computeProperties(this, mass_full, centerOfMass_full, inertia_full);
	}
	

	public enum IntLayout {
		Faces, Edges;

		/**
		 * Computes the start of the n-th primitive in the int array
		 * 
		 * @param n
		 * @param data
		 * @return
		 */
		public int startOf(int n, ConvexHullWrapperData data) {
			return startOf(n, data.faceCount, data.edgeCount);
		}

		private int startOf(int n, int faceCount, int edgeCount) {
			switch (this) {
			case Faces:
				Objects.checkIndex(n, faceCount);
				return FaceLayout.size() * n;
			case Edges:
				Objects.checkIndex(n, edgeCount);
				return FaceLayout.size() * faceCount + EdgeLayout.size() * n;
			default:
				throw new IllegalStateException();
			}
		}

		public static final int size(int faceCount, int edgeCount) {
			return faceCount * FaceLayout.size() + edgeCount * EdgeLayout.size();
		}
	}

	public enum FloatLayout {
		Vertices, FaceNormals, FaceCentroids, PlaneOffsets, BackupVertices, BackupFaceNormals, BackupFaceCentroids;

		/**
		 * Computes the start of the n-th primitive in the float array
		 * 
		 * @param n
		 * @param data
		 * @return
		 */
		public int startOf(int n, ConvexHullWrapperData data) {
			return startOf(n, data.vertexCount, data.faceCount);
		}

		private int startOf(int n, int vertexCount, int faceCount) {
			switch (this) {
			case Vertices:
				Objects.checkIndex(n, vertexCount);
				return 3 * n;
			case FaceNormals:
				Objects.checkIndex(n, faceCount);
				return 3 * vertexCount + 3 * n;
			case FaceCentroids:
				Objects.checkIndex(n, faceCount);
				return 3 * vertexCount + 3 * faceCount + 3 * n;
			case PlaneOffsets:
				Objects.checkIndex(n, faceCount);
				return 3 * vertexCount + 3 * faceCount + 3 * faceCount + n;
			case BackupVertices:
				Objects.checkIndex(n, vertexCount);
				return 3 * vertexCount + 3 * faceCount + 3 * faceCount + faceCount + 3 * n;
			case BackupFaceNormals:
				Objects.checkIndex(n, faceCount);
				return 3 * vertexCount + 3 * faceCount + 3 * faceCount + faceCount + 3 * vertexCount + 3 * n;
			case BackupFaceCentroids:
				Objects.checkIndex(n, faceCount);
				return 3 * vertexCount + 3 * faceCount + 3 * faceCount + faceCount + 3 * vertexCount + 3 * faceCount
						+ 3 * n;
			default:
				throw new IllegalStateException();
			}
		}

		public static final int size(int faceCount, int vertexCount) {
			return 2 * (3 * vertexCount + 3 * faceCount + 3 * faceCount) + faceCount;
		}

	}

	/**
	 * Layout of a face in the int array
	 * 
	 * @author Briac
	 *
	 */
	public enum FaceLayout {
		/**
		 * Index of its first edge
		 */
		Edge0;

		public int offset() {
			return this.ordinal();
		}

		public static final int size() {
			return 1;
		}
	}

	public enum EdgeLayout {
		tail, next, twin, face;

		int offset() {
			return this.ordinal();
		}

		static final int size() {
			return 4;
		}
	}

//	public float dot(int left, int right) {
//	return floatData[left + 0] * floatData[right + 0] + floatData[left + 1] * floatData[right + 1]
//			+ floatData[left + 2] * floatData[right + 2];
//}
	
	public float dot(FloatLayout type, int n, Vector3f right) {
		int index = type.startOf(n, this);
		return floatData[index + 0] * right.x + floatData[index + 1] * right.y + floatData[index + 2] * right.z;
	}
	
	public void sub(FloatLayout typeLeft, int nLeft, FloatLayout typeRight, int nRight, Vector3f dest) {
		int leftStart = typeLeft.startOf(nLeft, this);
		int rightStart = typeRight.startOf(nRight, this);
	
		dest.x = floatData[leftStart + 0] - floatData[rightStart + 0];
		dest.y = floatData[leftStart + 1] - floatData[rightStart + 1];
		dest.z = floatData[leftStart + 2] - floatData[rightStart + 2];
	}
	
	public void sub(FloatLayout typeLeft, int nLeft, Vector3f right, Vector3f dest) {
		int leftStart = typeLeft.startOf(nLeft, this);
	
		dest.x = floatData[leftStart + 0] - right.x;
		dest.y = floatData[leftStart + 1] - right.y;
		dest.z = floatData[leftStart + 2] - right.z;
	}
	
	public void get(FloatLayout type, int n, Vector3f dest) {
		int start = type.startOf(n, this);
	
		dest.x = floatData[start + 0];
		dest.y = floatData[start + 1];
		dest.z = floatData[start + 2];
	}
	
	public void getNormal(int face, Vector3f dest) {
		int index = FloatLayout.FaceNormals.startOf(face, this);
		dest.x = floatData[index + 0];
		dest.y = floatData[index + 1];
		dest.z = floatData[index + 2];
	}
	
	public void getCentroid(int faceIndex, Vector3f dest) {
		int index = FloatLayout.FaceCentroids.startOf(faceIndex, this);
		dest.x = floatData[index + 0];
		dest.y = floatData[index + 1];
		dest.z = floatData[index + 2];
	}
	
	public float getPlaneOffset(int face) {
		int index = FloatLayout.PlaneOffsets.startOf(face, this);
		return floatData[index];
	}
	
	public float signedDistance(Vector3f position, int face) {
		int index = FloatLayout.FaceNormals.startOf(face, this);
		float nx = floatData[index + 0];
		float ny = floatData[index + 1];
		float nz = floatData[index + 2];
		return position.x * nx + position.y * ny + position.z * nz - getPlaneOffset(face);
	}
	
	public int getClosestFace(Vector3f position) {
		float distance = Float.NEGATIVE_INFINITY;
		int referenceFace = 0;
		for (int f = 0; f < faceCount; f++) {
			float d = signedDistance(position, f);
			if (d > distance) {
				distance = d;
				referenceFace = f;
			}
		}
		return referenceFace;
	}
	
	/**
	 * 
	 * @param direction
	 * @return the face whose normal gives the minimum scalar product with direction
	 */
	public int getMostAntiParallelFace(Vector3f direction) {

		float minDot = Float.POSITIVE_INFINITY;
		int antiparallelFace = -1;

		for (int face = 0; face < faceCount; face++) {
			float dot = dot(FloatLayout.FaceNormals, face, direction);
			if (dot < minDot) {
				minDot = dot;
				antiparallelFace = face;
			}
		}

		return antiparallelFace;
	}
	
	public int getFaceEdge0(int face) {
		return intData[IntLayout.Faces.startOf(face, this) + FaceLayout.Edge0.offset()];
	}
	
	public int getEdgeTail(int edge) {
		return intData[IntLayout.Edges.startOf(edge, this) + EdgeLayout.tail.offset()];
	}
	
	public int getEdgeNext(int edge) {
		return intData[IntLayout.Edges.startOf(edge, this) + EdgeLayout.next.offset()];
	}
	
	public int getEdgeTwin(int edge) {
		return intData[IntLayout.Edges.startOf(edge, this) + EdgeLayout.twin.offset()];
	}
	
	public int getEdgeFace(int edge) {
		return intData[IntLayout.Edges.startOf(edge, this) + EdgeLayout.face.offset()];
	}
	
	public int getEdgeHead(int edge) {
		return getEdgeTail(getEdgeNext(edge));
	}
	
	public int getEdgeAdjacentFace(int edge) {
		return getEdgeFace(getEdgeTwin(edge));
	}
	
	public void getEdgeVec(int edge, Vector3f dest) {
		int head = getEdgeHead(edge);
		int tail = getEdgeTail(edge);
	
		sub(FloatLayout.Vertices, head, FloatLayout.Vertices, tail, dest);
	}
	
	public WrapperModelData asModel() {
		WrapperModelData data = new WrapperModelData();
		
		List<Integer> indices = new ArrayList<>();
		List<Vector3f> vertices = new ArrayList<>();
		asModel(indices, vertices);

		data.indices = new int[indices.size()];
		data.vertices = new float[vertices.size() * 3];
		
		for(int i=0; i<indices.size(); i++) {
			data.indices[i] = indices.get(i);
		}
		for(int i=0; i<vertices.size(); i++) {
			data.vertices[3*i+0] = vertices.get(i).x;
			data.vertices[3*i+1] = vertices.get(i).y;
			data.vertices[3*i+2] = vertices.get(i).z;
		}
		
		return data;
	}
	
	public void asModel(List<Integer> indices, List<Vector3f> vertices) {
		indices.clear();
		vertices.clear();

		int totalVertexCount = 0;

		int faceStart = IntLayout.Faces.startOf(0, this.faceCount, this.edgeCount);
		int edgeStart = IntLayout.Edges.startOf(0, this.faceCount, this.vertexCount);
		for (int f = 0; f < this.faceCount; f++) {

			int firstEdge = this.intData[faceStart + FaceLayout.size() * f + FaceLayout.Edge0.offset()];
			int edge = firstEdge;

			int indexVertexOnFace = 0;
			do {

				int baseVertexIndex = this.intData[edgeStart + EdgeLayout.size() * edge + EdgeLayout.tail.offset()];
				int startVertex = FloatLayout.BackupVertices.startOf(baseVertexIndex, this.vertexCount, this.faceCount);

				float x = this.floatData[startVertex + 0];
				float y = this.floatData[startVertex + 1];
				float z = this.floatData[startVertex + 2];
				vertices.add(new Vector3f(x, y, z));

				if (indexVertexOnFace >= 2) {// Triangulation basique de la face
					indices.add(totalVertexCount);
					indices.add(totalVertexCount + indexVertexOnFace - 1);
					indices.add(totalVertexCount + indexVertexOnFace);
				}

				edge = this.intData[edgeStart + EdgeLayout.size() * edge + EdgeLayout.next.offset()];
				indexVertexOnFace++;
			} while (edge != firstEdge);

			totalVertexCount = vertices.size();

		}

	}

}
