package cataclysm.wrappers;

import math.vector.Matrix4f;
import math.vector.Vector3f;

final class ConvexHullWrapperBuilder extends WrapperBuilder {

	final ConvexHullWrapperData data;
	float scale = 1.0f;

	ConvexHullWrapperBuilder(Matrix4f bodySpaceTransform, ConvexHullWrapperData data) {
		super(bodySpaceTransform);
		this.data = data;
	}

	@Override
	Wrapper build(RigidBody body, long ID) {
		return new ConvexHullWrapper(body, this.wrapperToBody, this.massProperties, data, scale, ID);
	}

	@Override
	public void scale(float scaleFactor) {
		this.scale *= scaleFactor;
		this.massProperties.scale(scaleFactor);
	}

}
