package cataclysm.wrappers;

import math.vector.Matrix4f;

/**
 * This class serves as a factory to give a {@link RigidBody} a {@link Wrapper}.
 * 
 * @author Briac
 *
 */
@SuppressWarnings("preview")
public abstract sealed class WrapperBuilder permits SphereBuilder, CapsuleBuilder, ConvexHullWrapperBuilder{

	protected final Transform wrapperToBody;
	protected final MassProperties massProperties;

	/**
	 * Package-private constructor, called by subclasses.
	 * 
	 * @param wrapperToBody
	 */
	WrapperBuilder(Matrix4f wrapperToBody) {
		this.wrapperToBody = new Transform(wrapperToBody);
		this.massProperties = new MassProperties(0, 0, false, 1);
	}

	/**
	 * @return The bodyspace transforms of the wrappers that will be built from this
	 *         factory.
	 */
	public Transform getTransform() {
		return wrapperToBody;
	}

	/**
	 * Sets the bodyspace transforms of the wrappers that will be built from this
	 * factory. Previously built wrappers will not be affected.
	 * 
	 * @param transform
	 */
	public void setTransform(Matrix4f transform) {
		this.wrapperToBody.loadFrom(transform);
	}

	/**
	 * The wrappers built from this factory can be solid or hollow. This only affects the
	 * inertia tensor of the rigid body. Previously built wrappers will not be affected.
	 * Wrappers are solid by default.
	 * 
	 * @param hollow
	 * @return this
	 */
	public WrapperBuilder setHollow(boolean hollow) {
		this.massProperties.setHollow(hollow);
		return this;
	}

	/**
	 * Changes the density of the wrappers built from this factory. <br>
	 * If the hollow parameter is true, this parameter is understood as the mass per
	 * surface unit.<br>
	 * If the hollow parameter is false, this parameter is understood as the mass
	 * per volume unit.
	 * 
	 * @param density
	 * @return this
	 */
	public WrapperBuilder setDensity(float density) {
		this.massProperties.setDensity(density);
		this.massProperties.setUseDensity(true);
		return this;
	}

	/**
	 * Changes the mass of the wrappers built from this factory. <br>
	 * If the hollow parameter is true, the density is computed as the ratio of the
	 * mass to the surface area.<br>
	 * If the hollow parameter is false, the density is computed as the ratio of the
	 * mass to the volume.
	 * 
	 * 
	 * @param mass
	 */
	public void setMass(float mass) {
		this.massProperties.setMass(mass);
		this.massProperties.setUseDensity(false);
	}

	/**
	 * Builds a new {@link Wrapper} for the given {@link RigidBody}.
	 * 
	 * @param body
	 * @param ID
	 * @return the newly built wrapper.
	 */
	abstract Wrapper build(RigidBody body, long ID);

	/**
	 * Applies a scaling on this builder. All {@link Wrapper}s built subsequently
	 * will be affected by this scaling. Previously built wrappers will not be
	 * affected. Note that the translation and rotation of the wrappers in bodyspace
	 * will stay unchanged. To achieve a true scaling, the translation vector should
	 * also be modified, see {@link #setTransform(Matrix4f)}.
	 * 
	 * @param scaleFactor
	 */
	public abstract void scale(float scaleFactor);

}
