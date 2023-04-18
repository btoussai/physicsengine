package cataclysm.wrappers;

import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.wrappers.ConvexHullWrapperData.FloatLayout;
import math.vector.Vector3f;

/**
 * A triangle represented as a convex hull.
 * 
 * @author Briac Toussaint
 *
 */
public final class TriangleAsHull extends ConvexHullWrapper {

	private TriangleAsHull() {
		super(buildBase());
	}

	private static ConvexHullWrapperData buildBase() {
		int vertexCount = 3;
		int edgeCount = 6;
		int faceCount = 2;

		// Edge0;
		// tail, next, prev, twin, face;

		short top_edge0 = 0;
		short bottom_edge0 = 1;

		short e12_tail = 0;
		short e12_next = 2;
		short e12_twin = 1;
		short e12_face = 0;

		short e21_tail = 1;
		short e21_next = 5;
		short e21_twin = 0;
		short e21_face = 1;

		short e23_tail = 1;
		short e23_next = 4;
		short e23_twin = 3;
		short e23_face = 0;

		short e32_tail = 2;
		short e32_next = 1;
		short e32_twin = 2;
		short e32_face = 1;

		short e31_tail = 2;
		short e31_next = 0;
		short e31_twin = 5;
		short e31_face = 0;

		short e13_tail = 0;
		short e13_next = 3;
		short e13_twin = 4;
		short e13_face = 1;

		short[] intData = { top_edge0, bottom_edge0,

				e12_tail, e12_next, e12_twin, e12_face,

				e21_tail, e21_next, e21_twin, e21_face,

				e23_tail, e23_next, e23_twin, e23_face,

				e32_tail, e32_next, e32_twin, e32_face,

				e31_tail, e31_next, e31_twin, e31_face,

				e13_tail, e13_next, e13_twin, e13_face };

		// Vertices, FaceNormals, FaceCentroids, PlaneOffsets, BackupVertices,
		// BackupFaceNormals, BackupFaceCentroids;
		float[] floatData = new float[3 * vertexCount + 3 * faceCount + 3 * faceCount + faceCount + 3 * vertexCount
				+ 3 * faceCount + 3 * faceCount];
		ConvexHullWrapperData data = new ConvexHullWrapperData(faceCount, edgeCount, vertexCount, intData, floatData,
				0, new PolyhedralMassProperties());

		return data;
	}

	public static TriangleAsHull buildNew() {
		return new TriangleAsHull();
	}

	public void setFrom(Triangle triangle) {
		// vertices
		data.floatData[0] = triangle.getV0(0);
		data.floatData[1] = triangle.getV0(1);
		data.floatData[2] = triangle.getV0(2);
		data.floatData[3] = triangle.getV1(0);
		data.floatData[4] = triangle.getV1(1);
		data.floatData[5] = triangle.getV1(2);
		data.floatData[6] = triangle.getV2(0);
		data.floatData[7] = triangle.getV2(1);
		data.floatData[8] = triangle.getV2(2);

		// normals
		data.floatData[9] = triangle.getNormal(0);
		data.floatData[10] = triangle.getNormal(1);
		data.floatData[11] = triangle.getNormal(2);
		data.floatData[12] = -data.floatData[9];
		data.floatData[13] = -data.floatData[10];
		data.floatData[14] = -data.floatData[11];

		// centroids
		float cx = (1.0f / 3.0f) * (data.floatData[0] + data.floatData[3] + data.floatData[6]);
		float cy = (1.0f / 3.0f) * (data.floatData[1] + data.floatData[4] + data.floatData[7]);
		float cz = (1.0f / 3.0f) * (data.floatData[2] + data.floatData[5] + data.floatData[8]);
		data.floatData[15] = data.floatData[18] = cx;
		data.floatData[16] = data.floatData[19] = cy;
		data.floatData[17] = data.floatData[20] = cz;

		// plane offsets
		data.floatData[21] = triangle.getPlaneOffset();
		data.floatData[22] = -triangle.getPlaneOffset();

		super.getCentroid().set(cx, cy, cz);
	}
	

	@Override
	public void getSupport(Vector3f direction, boolean negate, Vector3f dest) {
		float bestProjection = Float.NEGATIVE_INFINITY;

		dest.set(direction);
		if (negate) {
			dest.negate();
		}

		int start = FloatLayout.Vertices.startOf(0, data);
		int bestIndex = 0;
		for (int n = 0; n < data.vertexCount; n++) {
			float x = data.floatData[start + 3 * n + 0];
			float y = data.floatData[start + 3 * n + 1];
			float z = data.floatData[start + 3 * n + 2];
			float dot = x * dest.x + y * dest.y + z * dest.z;
			float projection = dot;
			if (projection > bestProjection) {
				bestProjection = projection;
				bestIndex = n;
			}
		}

		float x = data.floatData[start + 3 * bestIndex + 0];
		float y = data.floatData[start + 3 * bestIndex + 1];
		float z = data.floatData[start + 3 * bestIndex + 2];

		dest.set(x, y, z);

	}

	@Override
	public float getScale() {
		return 1.0f;
	}

	@Override
	public void transformVertexWorldSpaceToWrapperSpace(Vector3f v, Vector3f dest) {
		dest.set(v);
	}

	@Override
	public void transformVertexWrapperSpaceToWorldSpace(Vector3f v, Vector3f dest) {
		dest.set(v);
	}

	@Override
	public void transformNormalWorldSpaceToWrapperSpace(Vector3f n, Vector3f dest) {
		dest.set(n);
	}

	@Override
	public void transformNormalWrapperSpaceToWorldSpace(Vector3f n, Vector3f dest) {
		dest.set(n);
	}


}
