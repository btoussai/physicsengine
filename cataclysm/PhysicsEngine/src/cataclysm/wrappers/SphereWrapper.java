package cataclysm.wrappers;

import cataclysm.record.WrapperRepr;
import math.MatrixOps;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Defines a spherical {@link Wrapper} for a {@link RigidBody}.
 * 
 * @author Briac Toussaint
 *
 */
public final class SphereWrapper extends Wrapper {

	private float radius;

	SphereWrapper(RigidBody body, Transform wrapperToBody, MassProperties massProperties, float radius, long ID) {
		super(body, wrapperToBody, massProperties, radius, ID);
		this.radius = radius;
	}
	
	public SphereWrapper(RigidBody body, WrapperRepr w, long ID) {
		super(body, w, ID);
		this.radius = w.sphereRadius;
	}

	@Override
	public Type getType() {
		return Wrapper.Type.Sphere;
	}

	public float getRadius() {
		return radius;
	}

	public void getSupport(Vector3f direction, boolean negate, Vector3f dest) {
		dest.set(getCentroid());
	}

	@Override
	public void scale(float scaleFactor) {
		this.radius *= scaleFactor;
		super.scale(scaleFactor);
	}

	@Override
	public float computeInertia(Vector3f centerOfMass, Matrix3f inertia, PolyhedralMassProperties poly) {
		inertia.setIdentity();
		centerOfMass.set(0.0f, 0.0f, 0.0f);
		
		float surfaceArea = 4.0f * (float)Math.PI * radius * radius;
		float volume = (1.0f / 3.0f) * surfaceArea * radius;
		
		massProperties.setSurfaceArea(surfaceArea);
		massProperties.setVolume(volume);
		float mass = massProperties.computeMass();
		
		if (massProperties.isHollow()) {
			inertia.m00 = inertia.m11 = inertia.m22 = (2.0f / 3.0f) * radius * radius * mass;
		} else {
			inertia.m00 = inertia.m11 = inertia.m22 = (2.0f / 5.0f) * radius * radius * mass;
		}
		
		PolyhedralMassProperties.transformMassProperties(inertia, centerOfMass, massProperties, wrapperToBody.getTranslation(), wrapperToBody.getRotation(), 1.0f);
		
		return mass;
	}

	@Override
	protected void fill(WrapperRepr w) {
		super.fill(w);
		w.sphereRadius = radius;
	}

}
