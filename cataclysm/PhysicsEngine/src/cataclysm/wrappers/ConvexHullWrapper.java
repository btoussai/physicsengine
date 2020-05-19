package cataclysm.wrappers;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import cataclysm.record.WrapperRepr;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Repr�sente une enveloppe convexe pour les collisions.
 * 
 * @author Briac
 *
 */
public class ConvexHullWrapper extends Wrapper {

	public final int faceCount;
	public final int edgeCount;
	public final int vertexCount;
	public final short[] intData;
	public final float[] floatData;

	/**
	 * Le facteur d'échelle entre le modèle 3D et les coordonnées réelles du wrapper
	 */
	private float scale;

	/**
	 * Construit une nouvelle enveloppe convexe pour les collisions.
	 * 
	 * @param body
	 * @param wrapperToBody
	 * @param data
	 */
	protected ConvexHullWrapper(RigidBody body, Transform wrapperToBody, MassProperties massProperties,
			ConvexHullWrapperData data, long ID) {
		super(body, wrapperToBody, massProperties, data.maxRadius, ID);
		this.faceCount = data.faceCount;
		this.edgeCount = data.edgeCount;
		this.vertexCount = data.vertexCount;
		this.intData = Arrays.copyOf(data.intData, data.intData.length);
		this.floatData = Arrays.copyOf(data.floatData, data.floatData.length);
		this.scale = 1.0f;
	}

	public ConvexHullWrapper(RigidBody body, WrapperRepr w, long ID) {
		super(body, w, ID);
		this.faceCount = w.faceCount;
		this.edgeCount = w.edgeCount;
		this.vertexCount = w.vertexCount;
		this.intData = Arrays.copyOf(w.intData, w.intData.length);
		this.floatData = Arrays.copyOf(w.floatData, w.floatData.length);
		this.scale = 1.0f;
	}

	/**
	 * This constructor should only be used by {@link TriangleAsHull}
	 */
	protected ConvexHullWrapper(ConvexHullWrapperData w) {
		super();
		this.faceCount = w.faceCount;
		this.edgeCount = w.edgeCount;
		this.vertexCount = w.vertexCount;
		this.intData = Arrays.copyOf(w.intData, w.intData.length);
		this.floatData = Arrays.copyOf(w.floatData, w.floatData.length);
		this.scale = 1.0f;
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

	@Override
	public Type getType() {
		return Wrapper.Type.ConvexHull;
	}

	@Override
	public float computeInertia(Vector3f centerOfMass, Matrix3f inertia, PolyhedralMassProperties poly) {
		this.transform(wrapperToBody);

		float mass = poly.computeProperties(this, centerOfMass, inertia);

		Vector3f wrapperCenterOfMass = new Vector3f();
		wrapperToBody.invertTransformVertex(centerOfMass, wrapperCenterOfMass);
		// On indique la position du centre de masse en wrapper-space.
		super.placeCentroid(wrapperCenterOfMass);

		return mass;
	}

	@Override
	protected void fill(WrapperRepr w) {
		super.fill(w);
		throw new IllegalStateException("Not implemented");
	}

	public enum IntLayout {
		Faces, Edges;

		/**
		 * Computes the start of the n-th primitive in the int array
		 * 
		 * @param n
		 * @param hull
		 * @return
		 */
		public int startOf(int n, ConvexHullWrapper hull) {
			return startOf(n, hull.faceCount, hull.edgeCount);
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
		 * @param hull
		 * @return
		 */
		public int startOf(int n, ConvexHullWrapper hull) {
			return startOf(n, hull.vertexCount, hull.faceCount);
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

	@Override
	public void transform(Transform wrapperToWorld) {
		for (int n = 0; n < vertexCount; n++) {
			int src = FloatLayout.BackupVertices.startOf(n, this);
			int dest = FloatLayout.Vertices.startOf(n, this);
			wrapperToWorld.transformVertex(floatData, src, floatData, dest);
		}

		for (int n = 0; n < faceCount; n++) {
			int srcNormals = FloatLayout.BackupFaceNormals.startOf(n, this);
			int destNormals = FloatLayout.FaceNormals.startOf(n, this);
			wrapperToWorld.transformVector(floatData, srcNormals, floatData, destNormals);

			int srcCentroids = FloatLayout.BackupFaceCentroids.startOf(n, this);
			int destCentroids = FloatLayout.FaceCentroids.startOf(n, this);
			wrapperToWorld.transformVertex(floatData, srcCentroids, floatData, destCentroids);

			int offset_i = FloatLayout.PlaneOffsets.startOf(n, this);
			floatData[offset_i] = dot(destCentroids, destNormals);
		}

		super.transform(wrapperToWorld);
	}

	@Override
	public void scale(float scaleFactor) {

		Vector3f origin = super.getCentroidWrapperSpace();

		// On applique le changement d'échelle aux sommets.
		for (int n = 0; n < vertexCount; n++) {
			int index = FloatLayout.BackupVertices.startOf(n, vertexCount, faceCount);
			floatData[index + 0] = origin.x + (floatData[index + 0] - origin.x) * scaleFactor;
			floatData[index + 1] = origin.y + (floatData[index + 1] - origin.y) * scaleFactor;
			floatData[index + 2] = origin.z + (floatData[index + 2] - origin.z) * scaleFactor;
		}

		// Puis aux centres des faces
		for (int n = 0; n < faceCount; n++) {
			int index = FloatLayout.BackupFaceCentroids.startOf(n, vertexCount, faceCount);
			floatData[index + 0] = origin.x + (floatData[index + 0] - origin.x) * scaleFactor;
			floatData[index + 1] = origin.y + (floatData[index + 1] - origin.y) * scaleFactor;
			floatData[index + 2] = origin.z + (floatData[index + 2] - origin.z) * scaleFactor;
		}
		this.scale *= scaleFactor;

		super.scale(scaleFactor);
	}

	void translate(float x, float y, float z) {
		// On applique la translation aux sommets.
		for (int n = 0; n < vertexCount; n++) {
			int index = FloatLayout.BackupVertices.startOf(n, vertexCount, faceCount);
			floatData[index + 0] += x;
			floatData[index + 1] += y;
			floatData[index + 2] += z;
		}

		// Puis aux centres des faces
		for (int n = 0; n < faceCount; n++) {
			int index = FloatLayout.BackupFaceCentroids.startOf(n, vertexCount, faceCount);
			floatData[index + 0] += x;
			floatData[index + 1] += y;
			floatData[index + 2] += z;
		}
	}

	@Override
	public void getSupport(Vector3f direction, boolean negate, Vector3f dest) {
		float bestProjection = Float.NEGATIVE_INFINITY;

		int start = FloatLayout.Vertices.startOf(0, this);
		if (negate) {

			for (int n = 0; n < vertexCount; n++) {
				float x = floatData[start + 3 * n + 0];
				float y = floatData[start + 3 * n + 1];
				float z = floatData[start + 3 * n + 2];
				float dot = x * direction.x + y * direction.y + z * direction.z;
				float projection = -dot;
				if (projection > bestProjection) {
					bestProjection = projection;
					dest.set(x, y, z);
				}
			}

		} else {

			for (int n = 0; n < vertexCount; n++) {
				float x = floatData[start + 3 * n + 0];
				float y = floatData[start + 3 * n + 1];
				float z = floatData[start + 3 * n + 2];
				float dot = x * direction.x + y * direction.y + z * direction.z;
				float projection = dot;
				if (projection > bestProjection) {
					bestProjection = projection;
					dest.set(x, y, z);
				}
			}
		}

	}

	/**
	 * @return Le facteur d'echelle entre le mod�le et les sommets de l'enveloppe.
	 */
	public float getScale() {
		return scale;
	}

	public void asModel(List<Integer> indices, List<Vector3f> vertices) {
		indices.clear();
		vertices.clear();

		int totalVertexCount = 0;

		int faceStart = IntLayout.Faces.startOf(0, faceCount, edgeCount);
		int edgeStart = IntLayout.Edges.startOf(0, faceCount, vertexCount);
		for (int f = 0; f < faceCount; f++) {

			int firstEdge = intData[faceStart + FaceLayout.size() * f + FaceLayout.Edge0.offset()];
			int edge = firstEdge;

			int indexVertexOnFace = 0;
			do {

				int baseVertexIndex = intData[edgeStart + EdgeLayout.size() * edge + EdgeLayout.tail.offset()];
				int startVertex = FloatLayout.BackupVertices.startOf(baseVertexIndex, vertexCount, faceCount);

				float x = floatData[startVertex + 0];
				float y = floatData[startVertex + 1];
				float z = floatData[startVertex + 2];
				vertices.add(new Vector3f(x, y, z));

				if (indexVertexOnFace >= 2) {// Triangulation basique de la face
					indices.add(totalVertexCount);
					indices.add(totalVertexCount + indexVertexOnFace - 1);
					indices.add(totalVertexCount + indexVertexOnFace);
				}

				edge = intData[edgeStart + EdgeLayout.size() * edge + EdgeLayout.next.offset()];
				indexVertexOnFace++;
			} while (edge != firstEdge);

			totalVertexCount = vertices.size();

		}

	}

	private float dot(int left, int right) {
		return floatData[left + 0] * floatData[right + 0] + floatData[left + 1] * floatData[right + 1]
				+ floatData[left + 2] * floatData[right + 2];
	}

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
}
