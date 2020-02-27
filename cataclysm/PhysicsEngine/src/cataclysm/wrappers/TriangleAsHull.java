package cataclysm.wrappers;

import cataclysm.broadphase.staticmeshes.Triangle;
import math.vector.Vector3f;

/**
 * Repr�sente un triangle sous forme d'une enveloppe convexe.
 * Ceci permet de calculer les collisions entre les wrappers et les triangles
 * en r�utilisant les fonctions de collision contre une enveloppe convexe.
 * @author Briac
 *
 */
public class TriangleAsHull extends ConvexHullWrapper {

	private static final ConvexHullWrapperData data = buildBase();

	private TriangleAsHull() {
		super(data);
	}

	private static ConvexHullWrapperData buildBase() {
		ConvexHullWrapperFace top = new ConvexHullWrapperFace();
		top.index = 0;
		top.edge0 = 0;
		ConvexHullWrapperFace bottom = new ConvexHullWrapperFace();
		bottom.index = 1;
		bottom.edge0 = 1;

		ConvexHullWrapperHalfEdge e12 = new ConvexHullWrapperHalfEdge();
		e12.tail = 0;
		ConvexHullWrapperHalfEdge e21 = new ConvexHullWrapperHalfEdge();
		e21.tail = 1;

		e12.twin = 1;
		e21.twin = 0;

		ConvexHullWrapperHalfEdge e23 = new ConvexHullWrapperHalfEdge();
		e23.tail = 1;
		ConvexHullWrapperHalfEdge e32 = new ConvexHullWrapperHalfEdge();
		e32.tail = 2;

		e23.twin = 3;
		e32.twin = 2;

		ConvexHullWrapperHalfEdge e31 = new ConvexHullWrapperHalfEdge();
		e31.tail = 2;
		ConvexHullWrapperHalfEdge e13 = new ConvexHullWrapperHalfEdge();
		e13.tail = 0;

		e31.twin = 5;
		e13.twin = 4;

		e12.face = 0;
		e23.face = 0;
		e31.face = 0;

		e12.next = 2;
		e23.next = 4;
		e31.next = 0;

		e12.prev = 4;
		e23.prev = 0;
		e31.prev = 2;

		e21.face = 1;
		e32.face = 1;
		e13.face = 1;

		e21.next = 5;
		e13.next = 3;
		e32.next = 1;

		e21.prev = 3;
		e13.prev = 1;
		e32.prev = 5;

		e12.index = 0;
		e21.index = 1;
		e23.index = 2;
		e32.index = 3;
		e31.index = 4;
		e13.index = 5;

		ConvexHullWrapperFace[] faces = new ConvexHullWrapperFace[] { top, bottom };
		ConvexHullWrapperHalfEdge[] edges = new ConvexHullWrapperHalfEdge[] { e12, e21, e23, e32, e31, e13 };
		Vector3f[] vertices = new Vector3f[] { new Vector3f(), new Vector3f(), new Vector3f() };
		Vector3f[] faceNormals = new Vector3f[] { new Vector3f(), new Vector3f() };
		Vector3f[] faceCentroids = new Vector3f[] { new Vector3f(), new Vector3f() };
		
		ConvexHullWrapperData data = new ConvexHullWrapperData(faces, edges, vertices, faceNormals, faceCentroids);
		
		return data;
	}

	public static TriangleAsHull buildNew() {
		return new TriangleAsHull();
	}

	public void setFrom(Triangle triangle) {
		super.getData().vertices[0] = triangle.v1;
		super.getData().vertices[1] = triangle.v2;
		super.getData().vertices[2] = triangle.v3;

		super.getData().faceNormals[0].set(triangle.normal);
		Vector3f.negate(triangle.normal, super.getData().faceNormals[1]);

		super.getData().planeOffsets[0] = Vector3f.dot(triangle.v1, triangle.normal);
		super.getData().planeOffsets[1] = -super.getData().planeOffsets[0];

		this.getCentroid().set((1f / 3f) * (triangle.v1.x + triangle.v2.x + triangle.v3.x),
				(1f / 3f) * (triangle.v1.y + triangle.v2.y + triangle.v3.y),
				(1f / 3f) * (triangle.v1.z + triangle.v2.z + triangle.v3.z));
	}

}
