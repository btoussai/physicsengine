package cataclysm.wrappers;

import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Vector3f;

import math.MatrixOps;

/**
 * Représente une enveloppe sphérique utilisée pour les collisions.
 * 
 * @author Briac
 *
 */
public class SphereWrapper extends Wrapper {

	private float radius;

	SphereWrapper(RigidBody body, Transform wrapperToBody, MassProperties massProperties, float radius, long ID) {
		super(body, wrapperToBody, massProperties, radius, ID);
		this.radius = radius;
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
	public float computeInertia(Vector3f centerOfMass, Matrix3f inertia) {
		inertia.setIdentity();
		
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
		
		centerOfMass.set(wrapperToBody.getTranslation());
		
		MatrixOps.changeOfBasis(inertia, wrapperToBody.getRotation(), inertia);
		PolyhedralMassProperties.translateInertia(inertia, centerOfMass, mass);
		
		return mass;
	}

}
