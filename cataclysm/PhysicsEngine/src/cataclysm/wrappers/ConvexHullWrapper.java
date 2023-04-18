package cataclysm.wrappers;

import cataclysm.record.WrapperRepr;
import cataclysm.wrappers.ConvexHullWrapperData.FloatLayout;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Defines a convex polyhedra used as a collision shape for a {@link RigidBody}.
 * 
 * @author Briac Toussaint
 *
 */
public sealed class ConvexHullWrapper extends Wrapper permits TriangleAsHull {

	protected final ConvexHullWrapperData data;

	/**
	 * The scale factor between the backup data and the wrapper.
	 */
	private float scale;

	protected ConvexHullWrapper(RigidBody body, Transform wrapperToBody, MassProperties massProperties,
			ConvexHullWrapperData data, float scale, long ID) {
		super(body, wrapperToBody, massProperties, data.maxRadius, ID);
		this.data = data;
		this.scale = scale;
	}

	public ConvexHullWrapper(RigidBody body, WrapperRepr w, long ID) {
		super(body, w, ID);
		this.data = w.data;
		this.scale = 1.0f;
	}

	/**
	 * This constructor should only be used by {@link TriangleAsHull}
	 */
	protected ConvexHullWrapper(ConvexHullWrapperData w) {
		super();
		this.data = w;
		this.scale = 1.0f;
	}

	public ConvexHullWrapperData getConvexHullData() {
		return data;
	}

	@Override
	public Type getType() {
		return Wrapper.Type.ConvexHull;
	}

	@Override
	public float computeInertia(Vector3f centerOfMass, Matrix3f inertia, PolyhedralMassProperties poly) {
		if (massProperties.isHollow()) {
			inertia.load(data.inertia_hollow);
			centerOfMass.set(data.centerOfMass_hollow);
		} else {
			inertia.load(data.inertia_full);
			centerOfMass.set(data.centerOfMass_full);
		}

		massProperties.setVolume(data.mass_full.getVolume());
		massProperties.setSurfaceArea(data.mass_full.getSurfaceArea());
		massProperties.computeMass();

		PolyhedralMassProperties.transformMassProperties(inertia, centerOfMass, massProperties,
				wrapperToBody.getTranslation(), wrapperToBody.getRotation(), scale);

		super.placeCentroid(centerOfMass);

		return massProperties.getMass();
	}

	@Override
	protected void fill(WrapperRepr w) {
		super.fill(w);
		throw new IllegalStateException("Not implemented");
	}

	@Override
	public void scale(float scaleFactor) {
		this.scale *= scaleFactor;
		super.scale(scaleFactor);
	}

	@Override
	public void getSupport(Vector3f direction, boolean negate, Vector3f dest) {
		float bestProjection = Float.NEGATIVE_INFINITY;

		transformNormalWorldSpaceToWrapperSpace(direction, dest);
		if (negate) {
			dest.negate();
		}

		int start = FloatLayout.Vertices.startOf(0, data);
		int bestIndex = 0;
		for (int n = 0; n < data.vertexCount; n++) {
			float x = data.floatData[start + 3 * n + 0];
			float y = data.floatData[start + 3 * n + 1];
			float z = data.floatData[start + 3 * n + 2];
			float dot = x * dest.x + y * dest.y + z * dest.z;
			float projection = dot;
			if (projection > bestProjection) {
				bestProjection = projection;
				bestIndex = n;
			}
		}

		float x = data.floatData[start + 3 * bestIndex + 0];
		float y = data.floatData[start + 3 * bestIndex + 1];
		float z = data.floatData[start + 3 * bestIndex + 2];

		dest.set(x, y, z);
		transformVertexWrapperSpaceToWorldSpace(dest, dest);

	}

	/**
	 * @return The scale factor between this wrapper and the vertices data.
	 */
	public float getScale() {
		return scale;
	}

	public void transformVertexWorldSpaceToWrapperSpace(Vector3f v, Vector3f dest) {
		wrapperToWorld.invertTransformVertex(v, dest);
		dest.scale(1.0f / scale);
	}

	public void transformVertexWrapperSpaceToWorldSpace(Vector3f v, Vector3f dest) {
		dest.set(v.x * scale, v.y * scale, v.z * scale);
		wrapperToWorld.transformVertex(dest, dest);
	}

	public void transformNormalWorldSpaceToWrapperSpace(Vector3f n, Vector3f dest) {
		wrapperToWorld.invertTransformVector(n, dest);
	}

	public void transformNormalWrapperSpaceToWorldSpace(Vector3f n, Vector3f dest) {
		wrapperToWorld.transformVector(n, dest);
	}

}
