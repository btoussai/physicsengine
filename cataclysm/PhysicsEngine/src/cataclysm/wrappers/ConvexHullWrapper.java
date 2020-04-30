package cataclysm.wrappers;

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

	/**
	 * Les donn�es g�om�triques n�cessaires � la construction de l'enveloppe.
	 */
	private final ConvexHullWrapperData data;

	/**
	 * Construit une nouvelle enveloppe convexe pour les collisions.
	 * @param body
	 * @param wrapperToBody
	 * @param data
	 */
	protected ConvexHullWrapper(RigidBody body, Transform wrapperToBody, MassProperties massProperties, ConvexHullWrapperData data, long ID) {
		super(body, wrapperToBody, massProperties, data.maxRadius, ID);
		this.data = new ConvexHullWrapperData(data);
	}
	
	public ConvexHullWrapper(RigidBody body, WrapperRepr w, long ID) {
		super(body, w, ID);
		this.data = new ConvexHullWrapperData(w.data);
	}

	/**
	 * Ce constructeur ne doit �tre utilis� que par {@link TriangleAsHull}
	 */
	protected ConvexHullWrapper(ConvexHullWrapperData data) {
		super();
		this.data = new ConvexHullWrapperData(data);
	}

	/**
	 * Renvoit la face dont la normale minimise le produit scalaire avec
	 * "direction".
	 * 
	 * @param direction
	 * @return
	 */
	public ConvexHullWrapperFace getMostAntiParallelFace(Vector3f direction) {

		float minDot = Float.POSITIVE_INFINITY;
		ConvexHullWrapperFace antiparallelFace = null;

		for (ConvexHullWrapperFace face : data.faces) {
			float dot = Vector3f.dot(data.faceNormals[face.index], direction);
			if (dot < minDot) {
				minDot = dot;
				antiparallelFace = face;
			}
		}

		return antiparallelFace;
	}
	
	
	public ConvexHullWrapperData getData() {
		return data;
	}

	public ConvexHullWrapperFace[] getFaces() {
		return data.faces;
	}

	public ConvexHullWrapperHalfEdge[] getEdges() {
		return data.edges;
	}

	public Vector3f[] getVertices() {
		return data.vertices;
	}

	public Vector3f[] getFaceCentroids() {
		return data.faceCentroids;
	}

	@Override
	public Type getType() {
		return Wrapper.Type.ConvexHull;
	}

	@Override
	public void getSupport(Vector3f direction, boolean negate, Vector3f dest) {
		Vector3f supportPoint = null;
		float bestProjection = Float.NEGATIVE_INFINITY;
		if (negate) {

			for (Vector3f vertex : data.vertices) {
				float projection = -Vector3f.dot(vertex, direction);
				if (projection > bestProjection) {
					bestProjection = projection;
					supportPoint = vertex;
				}
			}

		} else {

			for (Vector3f vertex : data.vertices) {
				float projection = Vector3f.dot(vertex, direction);
				if (projection > bestProjection) {
					bestProjection = projection;
					supportPoint = vertex;
				}
			}
		}

		dest.set(supportPoint);
	}
	
	@Override
	public void transform(Transform wrapperToWorld) {
		data.tranform(wrapperToWorld);
		super.transform(wrapperToWorld);
	}


	@Override
	public void scale(float scaleFactor) {
		this.data.scale(scaleFactor, super.getCentroidWrapperSpace());
		super.scale(scaleFactor);
	}


	@Override
	public float computeInertia(Vector3f centerOfMass, Matrix3f inertia, PolyhedralMassProperties poly) {
		this.transform(wrapperToBody);
		
		float mass = poly.computeProperties(this, centerOfMass, inertia);
		
		Vector3f wrapperCenterOfMass = new Vector3f();
		wrapperToBody.invertTransformVertex(centerOfMass, wrapperCenterOfMass);
		//On indique la position du centre de masse en wrapper-space.
		super.placeCentroid(wrapperCenterOfMass);
		
		return mass;
	}

	@Override
	protected void fill(WrapperRepr w) {
		super.fill(w);
		w.data = data;
	}

}
