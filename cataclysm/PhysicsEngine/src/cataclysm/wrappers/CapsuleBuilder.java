package cataclysm.wrappers;

import math.vector.Matrix4f;

final class CapsuleBuilder extends WrapperBuilder{
	
	private float radius;
	private float halfLength;

	CapsuleBuilder(Matrix4f bodySpaceTransform, float radius, float halfLength) {
		super(bodySpaceTransform);
		this.radius = radius;
		this.halfLength = halfLength;
	}

	@Override
	Wrapper build(RigidBody body, long ID) {
		return new CapsuleWrapper(body, wrapperToBody, massProperties, radius, halfLength, ID);
	}

	@Override
	public void scale(float scaleFactor) {
		this.radius *= scaleFactor;
		this.halfLength *= scaleFactor;
		this.massProperties.scale(scaleFactor);
	}

}
