package cataclysm.wrappers;

import cataclysm.record.WrapperRepr;
import math.MatrixOps;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * This collision shape is a simple cylinder with round end caps.
 * 
 * @author Briac
 *
 */
public final class CapsuleWrapper extends Wrapper {

	private final TransformableVec3 center1 = new TransformableVec3();

	private final TransformableVec3 center2 = new TransformableVec3();

	private float radius;

	private float halfLength;

	CapsuleWrapper(RigidBody body, Transform wrapperToBody, MassProperties massProperties, float radius, float halfLength, long ID) {
		super(body, wrapperToBody, massProperties, halfLength + radius, ID);
		this.radius = radius;
		this.halfLength = halfLength;

		center1.getInputSpaceCoord().set(0, halfLength, 0);
		center2.getInputSpaceCoord().set(0, -halfLength, 0);
	}
	
	public CapsuleWrapper(RigidBody body, WrapperRepr w, long ID) {
		super(body, w, ID);
		this.radius = w.capsuleRadius;
		this.halfLength = w.halfLength;

		center1.getInputSpaceCoord().set(0, halfLength, 0);
		center2.getInputSpaceCoord().set(0, -halfLength, 0);
	}

	@Override
	public void transformCentroid() {
		this.center1.transformAsVertex(wrapperToWorld);
		this.center2.transformAsVertex(wrapperToWorld);
		super.transformCentroid();
	}

	@Override
	public Type getType() {
		return Wrapper.Type.Capsule;
	}

	public Vector3f getCenter1() {
		return center1.getOutputSpaceCoord();
	}

	public Vector3f getCenter2() {
		return center2.getOutputSpaceCoord();
	}

	public float getRadius() {
		return radius;
	}
	
	public float getHalfLength() {
		return halfLength;
	}

	@Override
	public void getSupport(Vector3f direction, boolean negate, Vector3f dest) {

		Vector3f.sub(getCenter2(), getCenter1(), dest);
		if (Vector3f.dot(dest, direction) < 0) {
			dest.set(negate ? getCenter2() : getCenter1());
		} else {
			dest.set(negate ? getCenter1() : getCenter2());
		}

	}

	@Override
	public void scale(float scaleFactor) {
		this.radius *= scaleFactor;
		this.halfLength *= scaleFactor;
		
		center1.getInputSpaceCoord().set(0, halfLength, 0);
		center2.getInputSpaceCoord().set(0, -halfLength, 0);
		
		super.scale(scaleFactor);
	}

	@Override
	public float computeInertia(Vector3f centerOfMass, Matrix3f inertia, PolyhedralMassProperties poly) {
		inertia.setIdentity();
		centerOfMass.set(0.0f, 0.0f, 0.0f);

		float r_plus_h = radius + halfLength;
		float r2 = radius * radius;
		float r3 = r2 * radius;
		float h2 = halfLength * halfLength;
		float h3 = h2 * halfLength;
		float rh = radius * halfLength;
		float r2h = r2 * halfLength;
		float rh2 = h2 * radius;
		
		float surfaceArea = 4.0f * (float) Math.PI * r2 + 4.0f * (float) Math.PI * rh;
		float volume = (4.0f / 3.0f) * (float) Math.PI * r3 + 2.0f * (float) Math.PI * r2h;
		
		massProperties.setSurfaceArea(surfaceArea);
		massProperties.setVolume(volume);
		float mass = massProperties.computeMass();
		
		if (massProperties.isHollow()) {
			inertia.m11 = (2f / 3f * radius + halfLength) * r2 / r_plus_h;
			inertia.m00 = inertia.m22 = (1f / 3f * h3 + rh2 + 3f / 2f * r2h + 0.25f * r3) / r_plus_h;
		} else {
			inertia.m11 = (halfLength + 8f / 15f * radius) / (4f / 3f * radius + 2 * halfLength) * r2;
			inertia.m00 = inertia.m22 = (2f / 3f * h3 + 4f / 3f * rh2 + 3f / 2f * r2h + 3f / 16f * r3)
					/ (4f / 3f * radius + 2 * halfLength);
		}

		inertia.m00 *= mass;
		inertia.m11 *= mass;
		inertia.m22 *= mass;
		
		PolyhedralMassProperties.transformMassProperties(inertia, centerOfMass, massProperties, wrapperToBody.getTranslation(), wrapperToBody.getRotation(), 1.0f);
		
		return mass;
	}

	@Override
	protected void fill(WrapperRepr w) {
		super.fill(w);
		w.capsuleRadius = radius;
		w.halfLength = halfLength;
	}

}
