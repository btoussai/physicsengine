package cataclysm.wrappers;

import cataclysm.broadphase.staticmeshes.Triangle;

/**
 * A triangle represented as a convex hull.
 * 
 * @author Briac Toussaint
 *
 */
public final class TriangleAsHull extends ConvexHullWrapper {

	private static final ConvexHullWrapperData data = buildBase();

	private TriangleAsHull() {
		super(data);
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
				0);

		return data;
	}

	public static TriangleAsHull buildNew() {
		return new TriangleAsHull();
	}

	public void setFrom(Triangle triangle) {
		// vertices
		floatData[0] = triangle.getV0(0);
		floatData[1] = triangle.getV0(1);
		floatData[2] = triangle.getV0(2);
		floatData[3] = triangle.getV1(0);
		floatData[4] = triangle.getV1(1);
		floatData[5] = triangle.getV1(2);
		floatData[6] = triangle.getV2(0);
		floatData[7] = triangle.getV2(1);
		floatData[8] = triangle.getV2(2);

		// normals
		floatData[9] = triangle.getNormal(0);
		floatData[10] = triangle.getNormal(1);
		floatData[11] = triangle.getNormal(2);
		floatData[12] = -floatData[9];
		floatData[13] = -floatData[10];
		floatData[14] = -floatData[11];

		// centroids
		float cx = (1.0f / 3.0f) * (floatData[0] + floatData[3] + floatData[6]);
		float cy = (1.0f / 3.0f) * (floatData[1] + floatData[4] + floatData[7]);
		float cz = (1.0f / 3.0f) * (floatData[2] + floatData[5] + floatData[8]);
		floatData[15] = floatData[18] = cx;
		floatData[16] = floatData[19] = cy;
		floatData[17] = floatData[20] = cz;

		// plane offsets
		floatData[21] = triangle.getPlaneOffset();
		floatData[22] = -triangle.getPlaneOffset();

		super.getCentroid().set(cx, cy, cz);
	}

}
