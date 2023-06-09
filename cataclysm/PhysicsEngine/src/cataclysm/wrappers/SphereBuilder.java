package cataclysm.wrappers;

import math.vector.Matrix4f;

final class SphereBuilder extends WrapperBuilder {
	
	private float radius;

	SphereBuilder(Matrix4f transform, float radius) {
		super(transform);
		this.radius = radius;
	}

	@Override
	Wrapper build(RigidBody body, long ID) {
		return new SphereWrapper(body, wrapperToBody, massProperties, radius, ID);
	}

	@Override
	public void scale(float scaleFactor) {
		this.radius *= scaleFactor;
		this.massProperties.scale(scaleFactor);
	}

}