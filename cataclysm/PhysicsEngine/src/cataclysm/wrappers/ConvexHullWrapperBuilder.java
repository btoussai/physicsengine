package cataclysm.wrappers;

import math.vector.Matrix4f;
import math.vector.Vector3f;

class ConvexHullWrapperBuilder extends WrapperBuilder {

	final ConvexHullWrapperData data;

	ConvexHullWrapperBuilder(Matrix4f bodySpaceTransform, ConvexHullWrapperData data) {
		super(bodySpaceTransform);
		this.data = data;
	}

	@Override
	Wrapper build(RigidBody body, long ID) {
		return new ConvexHullWrapper(body, this.wrapperToBody, this.massProperties, data, ID);
	}

	@Override
	public void scale(float scaleFactor) {
		this.data.scale(scaleFactor, new Vector3f());// can be false since we don't know where the center of mass is.
		this.massProperties.scale(scaleFactor);
	}

}
