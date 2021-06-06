package cataclysm.wrappers;

import java.util.ArrayList;

import cataclysm.Epsilons;
import cataclysm.broadphase.AABB;
import cataclysm.broadphase.BroadPhaseNode;
import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;
import cataclysm.datastructures.Identifier;
import cataclysm.record.WrapperRepr;
import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Defines a general convex shape for a {@link RigidBody}. Note that a
 * {@link RigidBody} can be composed of multiple wrappers, resulting in an
 * overall non-convex shape. Wrappers can be built from the static methods of
 * the {@link WrapperFactory}.
 * 
 * @see SphereWrapper
 * @see CapsuleWrapper
 * @see ConvexHullWrapper
 * @author Briac
 *
 */
@SuppressWarnings("preview")
public abstract sealed class Wrapper extends Identifier
		implements Comparable<Wrapper>permits SphereWrapper,CapsuleWrapper,ConvexHullWrapper {

	/**
	 * Defines the type of the wrapper.
	 * 
	 * @author Briac
	 *
	 */
	public enum Type {
		Sphere(1), Capsule(2), ConvexHull(Epsilons.MAX_CONTACTS);

		public final int maxContacts;

		private Type(int value) {
			this.maxContacts = value;
		}
	}

	/**
	 * The transformation from wrapper-space to body-space.
	 */
	protected final Transform wrapperToBody;

	/**
	 * The center of mass of the wrapper, in wrapper-space and in world-space.
	 */
	private final TransformableVec3 centroid;

	/**
	 * A reference to the parent rigid body.
	 */
	private final RigidBody body;

	/**
	 * A simple aligned-axis bounding box centered around the center of mass of the wrapper.
	 */
	private final BroadPhaseNode<Wrapper> node;
//	private int node;
//	private final AABB box;

	/**
	 * The maximum half-extent of this wrapper, i.e. its AABB has sides of length {@code maxRadius + padding}. 
	 */
	private float maxRadius;

	/**
	 * The mass properties of this wrapper (mass, volume, surface area, etc..)
	 */
	protected final MassProperties massProperties;

	/**
	 * A list of the contacts between this wrapper and static meshes.
	 */
	protected final ArrayList<AbstractSingleBodyContact> meshContacts = new ArrayList<AbstractSingleBodyContact>(1);

	/**
	 * A list of the contacts between this wrapper and other wrappers.
	 */
	protected final ArrayList<AbstractDoubleBodyContact> bodyContacts = new ArrayList<AbstractDoubleBodyContact>(1);


	protected Wrapper(RigidBody body, Transform wrapperToBody, MassProperties massProperties, float maxRadius,
			long ID) {
		super(ID);
		this.centroid = new TransformableVec3();
		this.wrapperToBody = new Transform(wrapperToBody);
		this.body = body;
		this.maxRadius = maxRadius;
		this.node = new BroadPhaseNode<Wrapper>(new AABB(), this);
//		this.node = -1;
//		this.box = new AABB();
		this.massProperties = new MassProperties(massProperties);
	}

	/**
	 * This constructor should only be used by {@link TriangleAsHull}
	 */
	protected Wrapper() {
		super(-1);
		this.centroid = new TransformableVec3();
		this.body = null;
		this.maxRadius = 0;
		this.node = null;
//		this.node = -1;
//		this.box = new AABB();
		this.wrapperToBody = null;
		this.massProperties = null;
	}

	/**
	 * Sets the position of the center of mass of this wrapper (in wrapper-space). <br>
	 * This function is called by {@link ConvexHullWrapper#computeInertia(Vector3f, Matrix3f, PolyhedralMassProperties)}
	 * when the inertia tensor of the rigid body is computed.
	 * 
	 * @param centroid
	 */
	protected void placeCentroid(Vector3f centroid) {
		this.centroid.getInputSpaceCoord().set(centroid);
	}

	/**
	 * Centers the bounding box of this wrapper around it.
	 * 
	 * 
	 * @param padding Some padding for the AABB.
	 */
	protected void placeBox(float padding) {
		Vector3f centroid = this.getCentroid();
		float halfSize = maxRadius + padding;
		node.getBox().set(centroid, halfSize);
//		box.set(centroid, halfSize);
	}

	/**
	 * @return The parent rigid body of this wrapper.
	 */
	public RigidBody getBody() {
		return body;
	}
	
	public BroadPhaseNode<Wrapper> getNode() {
		return node;
	}

//	public int getNode() {
//		return node;
//	}
//	
//	public void setNode(int node) {
//		this.node = node;
//	}

	/**
	 * @return The bounding box of this wrapper.
	 */
	public AABB getBox() {
//		return box;
		return node.getBox();
	}

	/**
	 * @return The center of mass of the wrapper in world-space
	 */
	public Vector3f getCentroid() {
		return centroid.getOutputSpaceCoord();
	}

	/**
	 * @return The center of mass of the wrapper in wrapper-space
	 */
	public Vector3f getCentroidWrapperSpace() {
		return centroid.getInputSpaceCoord();
	}

	/**
	 * @return The maximum half-extent of the wrapper.
	 */
	public float getMaxRadius() {
		return maxRadius;
	}

	/**
	 * @return A list of the contacts between this wrapper and static meshes.
	 */
	public ArrayList<AbstractSingleBodyContact> getMeshContacts() {
		return meshContacts;
	}

	/**
	 * @return A list of the contacts between this wrapper and other wrappers.
	 */
	public ArrayList<AbstractDoubleBodyContact> getBodyContacts() {
		return bodyContacts;
	}

	@Override
	public int compareTo(Wrapper other) {
		return this.getType().maxContacts - other.getType().maxContacts;
	}

	/**
	 * Recomputes the position of the wrapper in world-space.
	 * 
	 * @param wrapperToWorld The transformation from wrapper-space to world-space.
	 */
	public void transform(Transform wrapperToWorld) {
		this.centroid.transformAsVertex(wrapperToWorld);
	}

	/**
	 * @return The transformation from wrapper-space to body-space.
	 */
	public Transform getTransform() {
		return wrapperToBody;
	}

	/**
	 * @return the type of the wrapper
	 */
	public abstract Type getType();

	/**
	 * Computes the position of the farthest point on this wrapper in the given direction.
	 * 
	 * @param direction The direction vector, may not have unit-length. 
	 * @param negate    true if the search direction should be negated. The direction vector will not be affected.
	 * @param dest      The destination vector.
	 */
	public abstract void getSupport(Vector3f direction, boolean negate, Vector3f dest);

	/**
	 * Applies a scaling to this wrapper from its center of mass.
	 * 
	 * @param scaleFactor
	 */
	public void scale(float scaleFactor) {
		this.massProperties.scale(scaleFactor);
		float padding = 0.5f * (getBox().minX + getBox().maxX) - this.maxRadius;
		this.maxRadius *= scaleFactor;
		placeBox(padding);
	}

	/**
	 * Computes the inertia tensor and the center of mass of this wrapper in bodyspace.
	 * The inertia tensor is computed about the origin, not about the center of mass!
	 * The mass properties of the wrapper are also updated (mass, volume, surface area).
	 * 
	 * @param centerOfMass A destination vector for the center of mass.
	 * @param inertia      A destination matrix for the inertia tensor.
	 * @param poly         A helper class to compute the inertia tensor.
	 * @return the mass of the wrapper
	 */
	abstract float computeInertia(Vector3f centerOfMass, Matrix3f inertia, PolyhedralMassProperties poly);

	/**
	 * @return la masse, le volume, etc... de cette enveloppe.
	 */
	public MassProperties getMassProperties() {
		return massProperties;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder("Wrapper " + getType() + " : [ ");
		sb.append(super.toString());
		sb.append(" MeshContacts:" + meshContacts.size());
		sb.append(" BodyContacts:" + bodyContacts.size() + " ]");
		return sb.toString();
	}

	public Wrapper(RigidBody body, WrapperRepr w, long ID) {
		super(ID);

		this.centroid = new TransformableVec3(w.centroid);
		this.wrapperToBody = new Transform(w.wrapperToBody);
		this.body = body;
		this.maxRadius = w.maxRadius;
		this.node = new BroadPhaseNode<Wrapper>(new AABB(), this);
//		this.node = -1;
//		this.box = new AABB();
		this.massProperties = new MassProperties(w.massProperties);
	}

	protected void fill(WrapperRepr w) {
		w.wrapperToBody.loadFrom(wrapperToBody);
		w.centroid.set(centroid);
		w.maxRadius = maxRadius;
		w.massProperties.set(massProperties);
		w.type = getType().ordinal();
	}

}
