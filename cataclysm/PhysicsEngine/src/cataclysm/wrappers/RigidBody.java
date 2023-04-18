package cataclysm.wrappers;

import java.util.ArrayList;

import cataclysm.CollisionFilter;
import cataclysm.DefaultCollisionFilter;
import cataclysm.DefaultParameters;
import cataclysm.Epsilons.Sleep;
import cataclysm.annotations.Internal;
import cataclysm.annotations.ReadOnly;
import cataclysm.PhysicsWorld;
import cataclysm.constraints.AnchorPoint;
import cataclysm.contact_creation.ContactProperties;
import cataclysm.datastructures.IDGenerator;
import cataclysm.datastructures.Identifier;
import cataclysm.record.RigidBodyRepr;
import cataclysm.record.RigidBodyState;
import cataclysm.record.WrapperRepr;
import math.MatrixOps;
import math.vector.Matrix3f;
import math.vector.Matrix4f;
import math.vector.Vector3f;

/**
 * This class defines a rigid body. All dynamic objects in a
 * {@link PhysicsWorld} are rigid bodies. The actual collision shape of a rigid
 * body can be created from several convex primitives called {@link Wrapper}s.
 * <br>
 * 
 * A rigid body has two reference frames. The first one is called "body-space",
 * whose associated transform can be obtained with
 * {@link #getOriginTransform()}. The second one is barycentric, whose
 * associated transform can be obtained with {@link #getBarycentricTransform()}.
 * The center of mass is the zero vector in the barycentric reference frame and
 * the inertia tensor is diagonal. The physics engine only cares about the
 * barycentric reference frame whereas the programmer prefers the body-space
 * reference frame since the position of each wrapper is defined in body-space.
 * <br>
 * 
 * Both of these transforms are updated and managed by the physics engine and
 * must not be modified manually. They can however be read from without any
 * worries. For instance, the object simulated by a rigid body can be rendered
 * using the body-space transform. <br>
 * 
 * @author Briac Toussaint
 *
 */
public class RigidBody extends Identifier {

	/**
	 * A set of flags which customize the dynamic behavior of a rigid body.
	 * 
	 * @author Briac Toussaint
	 *
	 */
	public enum SpecialFlags {

		/**
		 * Indicates if this rigid body is affected by external forces such as gravity.
		 */
		EXTERNAL_FORCES,
		/**
		 * Indicates if this rigid body is allowed to rotate or not.
		 */
		ROTATION_BLOCKED,

		/**
		 * Indicates if the velocity should be computed as the integral of the
		 * acceleration or if it should be read from a recording.
		 */
		SKIP_INTEGRATION;

		byte setTrue(int flags) {
			return (byte) (flags | (1 << this.ordinal()));
		}

		byte setFalse(int flags) {
			return (byte) (flags & ~(1 << this.ordinal()));
		}

		void set(RigidBody b, boolean bool) {
			if (bool) {
				b.flags = setTrue(b.flags);
			} else {
				b.flags = setFalse(b.flags);
			}
		}

		boolean get(int flags) {
			return (flags & (1 << this.ordinal())) != 0;
		}
	}

	/**
	 * The position and rotation of the body in world-space.
	 */
	private final Transform bodyToWorld;

	/**
	 * The position and rotation of the reference frame attached to the center of
	 * mass, in world-space. The inertia tensor is diagonal in that reference frame.
	 */
	private final Transform barycentricToWorld = new Transform();

	/**
	 * The velocity of the center of mass in m/s (world-space).
	 */
	private final Vector3f velocity = new Vector3f();

	/**
	 * The angular velocity at the center of mass in radians/s (world-space)
	 */
	private final Vector3f angularVelocity = new Vector3f();

	/**
	 * The friction and elasticity of the object.
	 */
	private final ContactProperties contactProperties;

	/**
	 * The inverse of the mass: 1.0f / mass.
	 */
	private float inv_mass;

	/**
	 * The inertia moments of the object divided by its mass, in increasing order.
	 */
	private final Vector3f inertia = new Vector3f();

	/**
	 * The inverse of the inertia tensor, in world-space. <br>
	 * More specifically:<br>
	 * <br>
	 * inv_Iws = Orientation^T * diag( 1.0f / {@link #inertia} ) * Orientation
	 */
	private final Matrix3f inv_Iws = new Matrix3f();

	/**
	 * The collision shapes of this object.
	 */
	private final ArrayList<Wrapper> wrappers;

	/**
	 * The anchor points of the constraints attached to that object.
	 */
	private final ArrayList<AnchorPoint> anchorPoints;

	/**
	 * A pseudo velocity, which does not persist across frames.
	 */
	private final Vector3f pseudoVel = new Vector3f();

	/**
	 * A pseudo angular velocity, which does not persist across frames.
	 */
	private final Vector3f pseudoAngVel = new Vector3f();

	/**
	 * A mask to filter collisions with other rigid bodies.
	 */
	private int mask = 0xFFFFFFFF;

	/**
	 * A category to filter collisions with other rigid bodies.
	 */
	private int category = 0xFFFFFFFF;

	/**
	 * true if the rigid body is at rest.
	 */
	private boolean sleeping = false;

	/**
	 * The number of frames during which the rigid body has not moved significantly.
	 * The rigid body is put at rest when this counter is above
	 * {@link Sleep#FRAMES_SPENT_AT_REST} (if {@link Sleep#SLEEPING_ALLOWED} is
	 * true).
	 */
	private int sleepCounter = 0;

	/**
	 * A set of flags for this rigid body.
	 * 
	 * @see SpecialFlags
	 */
	private byte flags;

	RigidBody(Matrix4f transform, DefaultParameters params, IDGenerator bodyGenerator, IDGenerator wrapperGenerator,
			PolyhedralMassProperties poly, WrapperBuilder... builders) {
		super(bodyGenerator.nextID());
		this.bodyToWorld = new Transform(transform);
		this.wrappers = new ArrayList<Wrapper>(builders.length);
		this.anchorPoints = new ArrayList<AnchorPoint>(0);

		for (int i = 0; i < builders.length; i++) {
			this.wrappers.add(builders[i].build(this, wrapperGenerator.nextID()));
		}

		computeMassProperties(poly);

		updateTransforms();

		for (Wrapper wrapper : wrappers) {
			wrapper.placeBox(params.getPadding());
		}

		contactProperties = new ContactProperties(params.getContactProperties());
		setExternalForces(params.isGravity());
	}

	/**
	 * Updates the transforms of the reference frames of the rigid body. The
	 * position of the anchor points of the constraints in world-space will also be
	 * updated.
	 * 
	 * This function is called by the physics engine and must not be called
	 * manually.
	 * 
	 * @see #getOriginTransform()
	 * @see #getBarycentricTransform()
	 * @see #getAnchorPoints()
	 */
	@Internal
	public void updateTransforms() {
		if (!isRotationBlocked()) {
			float I1 = 1.0f / inertia.x;
			float I2 = 1.0f / inertia.y;
			float I3 = 1.0f / inertia.z;

			MatrixOps.changeOfBasis(I1, I2, I3, barycentricToWorld.getRotation(), inv_Iws);
		} else {
			inv_Iws.setZero();
		}

		for (int i = 0; i < wrappers.size(); i++) {
			Wrapper wrapper = wrappers.get(i);
			Transform.compose(bodyToWorld, wrapper.getWrapperToBodyTransform(), wrapper.getWrapperToWorldTransform());
			wrapper.transformCentroid();
		}

		for (int i = 0; i < anchorPoints.size(); i++) {
			AnchorPoint point = anchorPoints.get(i);
			bodyToWorld.transformVertex(point.getBodySpacePosition(), point.getWorldSpacePosition());
		}

	}

	/**
	 * Recomputes the mass properties of the rigid body (i.e its mass and inertia tensor). <br>
	 * This method is applied automatically when a rigid body is created. However, this method must be called explicitly under the following circumstances: <br>
	 * 
	 * <li> One or more wrappers have been added to or removed from this rigid body after its creation. </li>
	 * <li> One or more wrappers of this rigid body have been moved, rotated or scaled. </li>
	 * <li> The mass properties of one or more wrappers of this rigid body have changed. </li>
	 * 
	 * @param poly
	 */
	public void computeMassProperties(PolyhedralMassProperties poly) {
		float mass = 0;
		Matrix3f bodyInertia = new Matrix3f();
		bodyInertia.setZero();
		Vector3f bodyCenterOfMass = new Vector3f();
		Matrix3f wrapperInertia = new Matrix3f();
		Vector3f wrapperCenterOfMass = new Vector3f();
		for (Wrapper wrapper : wrappers) {
			float wrapperMass = wrapper.computeInertia(wrapperCenterOfMass, wrapperInertia, poly);
			bodyCenterOfMass.translate(wrapperMass * wrapperCenterOfMass.x, wrapperMass * wrapperCenterOfMass.y,
					wrapperMass * wrapperCenterOfMass.z);
			Matrix3f.add(bodyInertia, wrapperInertia, bodyInertia);
			mass += wrapperMass;
		}

		inv_mass = 1.0f / mass;
		bodyCenterOfMass.scale(inv_mass);
		PolyhedralMassProperties.translateInertiaToCenterOfMass(bodyInertia, bodyCenterOfMass, mass);

		//Compute the barycentricToBody transform
		MatrixOps.eigenValues(bodyInertia, inertia, this.barycentricToWorld.getRotation());
		inertia.scale(inv_mass);
		this.barycentricToWorld.getTranslation().set(bodyCenterOfMass);
		
		//Compose with bodyToWorld to obtain barcentricToWorld
		Transform.compose(bodyToWorld, barycentricToWorld, barycentricToWorld);
	}

	/**
	 * Applique un changement d'échelle sur ce rigidbody. Le changement d'echelle
	 * est appliqué au niveau du centre de masse et concerne tous les wrappers. Il
	 * n'est pas nécessaire d'appeler
	 * {@link #computeMassProperties(PolyhedralMassProperties poly)} après cette
	 * fonction.<br>
	 * <br>
	 * 
	 * La fonction {@link #updateTransforms()} est automatiquement appelée
	 * à la fin de la fonction pour finir la mise à jour de l'enveloppe.
	 * 
	 * @param scaleFactor
	 */
	public void scale(float scaleFactor) {

		this.inv_mass /= scaleFactor;
		this.inertia.scale(scaleFactor * scaleFactor);

		// On modifie la position du repère body-space par rapport au repère
		// barycentric-space.
		Vector3f v0 = barycentricToWorld.getTranslation();
		Vector3f v1 = bodyToWorld.getTranslation();
		v1.x = v0.x + (v1.x - v0.x) * scaleFactor;
		v1.y = v0.y + (v1.y - v0.y) * scaleFactor;
		v1.z = v0.z + (v1.z - v0.z) * scaleFactor;

		for (Wrapper wrapper : wrappers) {
			wrapper.scale(scaleFactor);// On applique le changement d'échelle sur l'enveloppe.

			// Puis on approche/éloigne le centre de masse de l'enveloppe en
			// barycentric-space
			wrapper.getWrapperToBodyTransform().getTranslation().scale(scaleFactor);

			Vector3f centroid = wrapper.getCentroidWrapperSpace();
			float dx = centroid.x * (scaleFactor - 1.0f);
			float dy = centroid.y * (scaleFactor - 1.0f);
			float dz = centroid.z * (scaleFactor - 1.0f);
			centroid.translate(dx, dy, dz);

			// In case the wrapper is a hull, we have to translate everything so that the
			// vertices stay at the same relative position with respect to the center of
			// mass.
//			if (wrapper instanceof ConvexHullWrapper) {
//				((ConvexHullWrapper) wrapper).translate(dx, dy, dz);
//			}
		}

		setSleeping(false);
		setSleepCounter(0);

		updateTransforms();
	}

	/**
	 * @return True if the body is subject to external forces such as gravity.
	 */
	public boolean isExternalForces() {
		return SpecialFlags.EXTERNAL_FORCES.get(flags);
	}

	/**
	 * Defines is this body is subject to external forces such as gravity.
	 * 
	 * @param externalForces
	 */
	public void setExternalForces(boolean externalForces) {
		SpecialFlags.EXTERNAL_FORCES.set(this, externalForces);
	}

	/**
	 * Translates the object to a new location.
	 * 
	 * @param translation
	 */
	public void translate(Vector3f translation) {
		barycentricToWorld.translate(translation);
		bodyToWorld.translate(translation);
	}

	/**
	 * Translates the object to a new location.
	 * 
	 * @param x
	 * @param y
	 * @param z
	 */
	public void translate(float x, float y, float z) {
		barycentricToWorld.translate(x, y, z);
		bodyToWorld.translate(x, y, z);
	}

	/**
	 * Applies a rotation about the center of mass of the rigid body.
	 * 
	 * @param rotation
	 */
	public void rotateAboutCenterOfMass(Matrix3f rotation) {
		barycentricToWorld.rotateLeft(rotation);
		bodyToWorld.rotateLeft(rotation);
		Vector3f v0 = barycentricToWorld.getTranslation();
		Vector3f v1 = bodyToWorld.getTranslation();
		float x = v1.x - v0.x;
		float y = v1.y - v0.y;
		float z = v1.z - v0.z;

		v1.x = v0.x + rotation.m00 * x + rotation.m10 * y + rotation.m20 * z;
		v1.y = v0.y + rotation.m01 * x + rotation.m11 * y + rotation.m21 * z;
		v1.z = v0.z + rotation.m02 * x + rotation.m12 * y + rotation.m22 * z;
	}

	/**
	 * Applies a compound rotation and translation at the center of mass of the
	 * rigid body.
	 * 
	 * @param transform
	 */
	public void transformCenterOfMass(Matrix4f transform) {
		barycentricToWorld.rotateLeft(transform);
		bodyToWorld.rotateLeft(transform);
		Vector3f v0 = barycentricToWorld.getTranslation();
		Vector3f v1 = bodyToWorld.getTranslation();
		float x = v1.x - v0.x;
		float y = v1.y - v0.y;
		float z = v1.z - v0.z;

		v1.x = v0.x + transform.m00 * x + transform.m10 * y + transform.m20 * z;
		v1.y = v0.y + transform.m01 * x + transform.m11 * y + transform.m21 * z;
		v1.z = v0.z + transform.m02 * x + transform.m12 * y + transform.m22 * z;

		if (transform.m30 != 0.0f || transform.m31 != 0.0f || transform.m32 != 0.0f)
			translate(transform.m30, transform.m31, transform.m32);
	}

	/**
	 * Applies a rotation at the origin of the rigid body.
	 * 
	 * @param rotation
	 */
	public void rotateAboutOrigin(Matrix3f rotation) {
		bodyToWorld.rotateLeft(rotation);
		barycentricToWorld.rotateLeft(rotation);
		Vector3f v0 = bodyToWorld.getTranslation();
		Vector3f v1 = barycentricToWorld.getTranslation();
		float x = v1.x - v0.x;
		float y = v1.y - v0.y;
		float z = v1.z - v0.z;

		v1.x = v0.x + rotation.m00 * x + rotation.m10 * y + rotation.m20 * z;
		v1.y = v0.y + rotation.m01 * x + rotation.m11 * y + rotation.m21 * z;
		v1.z = v0.z + rotation.m02 * x + rotation.m12 * y + rotation.m22 * z;
	}

	/**
	 * Applies a compound rotation and translation at the origin of the rigid body.
	 * 
	 * @param transform
	 */
	public void transformOrigin(Matrix4f transform) {
		bodyToWorld.rotateLeft(transform);
		barycentricToWorld.rotateLeft(transform);
		Vector3f v0 = bodyToWorld.getTranslation();
		Vector3f v1 = barycentricToWorld.getTranslation();
		float x = v1.x - v0.x;
		float y = v1.y - v0.y;
		float z = v1.z - v0.z;

		v1.x = v0.x + transform.m00 * x + transform.m10 * y + transform.m20 * z;
		v1.y = v0.y + transform.m01 * x + transform.m11 * y + transform.m21 * z;
		v1.z = v0.z + transform.m02 * x + transform.m12 * y + transform.m22 * z;

		if (transform.m30 != 0.0f || transform.m31 != 0.0f || transform.m32 != 0.0f)
			translate(transform.m30, transform.m31, transform.m32);
	}

	/**
	 * @return The position of the center of mass (the origin of the barycentric
	 *         reference-frame) in world-space. Must not be modified manually.<br>
	 *         Use {@link #translate(Vector3f)} to move the object to a new
	 *         location. Use {@link #transformCenterOfMass(Matrix4f)} or
	 *         {@link #transformOrigin(Matrix4f)} to apply a rotation and
	 *         translation.
	 */
	@ReadOnly
	public Vector3f getPosition() {
		return barycentricToWorld.getTranslation();
	}

	/**
	 * @return The rotation of the barycentric reference frame in world-space.
	 */
	@ReadOnly
	public Matrix3f getOrientation() {
		return barycentricToWorld.getRotation();
	}

	/**
	 * @return The transform of the barycentric reference frame in world-space.
	 */
	@ReadOnly
	public Transform getBarycentricTransform() {
		return barycentricToWorld;
	}

	/**
	 * @return The origin of the rigid-body in world-space. Use
	 *         {@link #translate(Vector3f)} to move the object to a new location.
	 */
	@ReadOnly
	public Vector3f getOriginPosition() {
		return bodyToWorld.getTranslation();
	}

	/**
	 * @return The rotation of the rigid body in world-space.
	 */
	@ReadOnly
	public Matrix3f getOriginOrientation() {
		return bodyToWorld.getRotation();
	}

	/**
	 * @return The position and rotation of the body in world-space.
	 */
	@ReadOnly
	public Transform getOriginTransform() {
		return bodyToWorld;
	}

	/**
	 * @return The velocity of the center of mass in m/s (world-space).
	 */
	public Vector3f getVelocity() {
		return velocity;
	}

	/**
	 * @return The angular velocity at the center of mass in radians/s (world-space)
	 */
	public Vector3f getAngularVelocity() {
		return angularVelocity;
	}

	@ReadOnly
	@Internal
	public Vector3f getPseudoVelocity() {
		return pseudoVel;
	}

	@ReadOnly
	@Internal
	public Vector3f getPseudoAngularVelocity() {
		return pseudoAngVel;
	}

	/**
	 * @return The inverse of the mass of the object: {@code 1.0f / mass}
	 */
	public float getInvMass() {
		return inv_mass;
	}

	/**
	 * Sets the inverse mass. <br>
	 * A rigid body with an inverse mass of zero (i.e. its mass is infinite) will
	 * remain unaffected by collisions. It will still be affected by external forces
	 * though. <br>
	 * A constraint applied on two objects having an infinite mass is undefined
	 * behavior.
	 * 
	 * @param inv_mass The inverse mass of the object.
	 */
	public void setInvMass(float inv_mass) {
		this.inv_mass = inv_mass;
	}

	/**
	 * @return the contact properties of the object.
	 */
	public ContactProperties getContactProperties() {
		return contactProperties;
	}

	/**
	 * Two rigid bodies A and B can collide if and only if: <br>
	 * {@code (maskA & categoryB) != 0 && (maskB & categoryA) != 0} by default. <br>
	 * This behavior can be modified by specifing a custom {@link CollisionFilter}.
	 * 
	 * @return The bitmask of this object.
	 */
	public int getMask() {
		return mask;
	}

	/**
	 * Sets the bitmask of this object. Two rigid bodies A and B can collide if and
	 * only if: <br>
	 * {@code (maskA & categoryB) != 0 && (maskB & categoryA) != 0} by default. <br>
	 * This behavior can be modified by specifing a custom {@link CollisionFilter}.
	 * 
	 * 
	 * @param mask The bitmask of this object.
	 */
	public void setMask(int mask) {
		this.mask = mask;
	}

	/**
	 * Two rigid bodies A and B can collide if and only if: <br>
	 * {@code (maskA & categoryB) != 0 && (maskB & categoryA) != 0} by default. <br>
	 * This behavior can be modified by specifing a custom {@link CollisionFilter}.
	 * 
	 * @return The category of this object.
	 */
	public int getCategory() {
		return category;
	}

	/**
	 * Sets the category of this object. Two rigid bodies A and B can collide if and
	 * only if: <br>
	 * {@code (maskA & categoryB) != 0 && (maskB & categoryA) != 0} by default. <br>
	 * This behavior can be modified by specifing a custom {@link CollisionFilter}.
	 * 
	 * @param category
	 */
	public void setCategory(int category) {
		this.category = category;
	}

	/**
	 * @return true if the rigid body is at rest.
	 */
	public boolean isSleeping() {
		return sleeping;
	}

	/**
	 * Sets the sleeping state of a rigid body.
	 * 
	 * @param sleeping true if the rigid body is at rest.
	 */
	public void setSleeping(boolean sleeping) {
		this.sleeping = sleeping;
	}

	/**
	 * @return The number of frames during which the rigid body has not moved
	 *         significantly.The rigid body is put at rest when this counter is
	 *         above Sleep.FRAMES_SPENT_AT_REST (if Sleep.SLEEPING_ALLOWED istrue).
	 * 
	 */
	public int getSleepCounter() {
		return sleepCounter;
	}

	/**
	 * Sets the number of frames during which the rigid body has not moved
	 * significantly.The rigid body is put at rest when this counter is above
	 * Sleep.FRAMES_SPENT_AT_REST (if Sleep.SLEEPING_ALLOWED istrue).
	 * 
	 * @param sleepCounter
	 */
	public void setSleepCounter(int sleepCounter) {
		this.sleepCounter = sleepCounter;
	}

	/**
	 * A rigid body whose rotation is blocked will have an infinite inertia tensor
	 * (the actual value remains unchanged). Its angular velocity will be conserved
	 * even if it collides with another object.
	 * 
	 * @return true if this rigid body is allowed to rotate.
	 */
	public boolean isRotationBlocked() {
		return SpecialFlags.ROTATION_BLOCKED.get(flags);
	}

	/**
	 * A rigid body whose rotation is blocked will have an infinite inertia tensor
	 * (the actual value remains unchanged). Its angular velocity will be conserved
	 * even if it collides with another object.
	 * 
	 * @param rotationBlocked
	 */
	public void setRotationBlocked(boolean rotationBlocked) {
		SpecialFlags.ROTATION_BLOCKED.set(this, rotationBlocked);
	}

	/**
	 * Indicates if the velocity should be computed as the integral of the
	 * acceleration or if it should be read from a recording.
	 * 
	 * @return
	 */
	public boolean isSkipIntegration() {
		return SpecialFlags.SKIP_INTEGRATION.get(flags);
	}

	/**
	 * Indicates if the velocity should be computed as the integral of the
	 * acceleration or if it should be read from a recording.
	 * 
	 * @param skipIntegration
	 */
	public void setSkipIntegration(boolean skipIntegration) {
		SpecialFlags.SKIP_INTEGRATION.set(this, skipIntegration);
	}

	byte getFlags() {
		return flags;
	}

	/**
	 * @return The inverse of the inertia tensor, in world-space.
	 */
	@ReadOnly
	@Internal
	public Matrix3f getInvIws() {
		return inv_Iws;
	}

	/**
	 * @return The inertia moments of the object divided by its mass, in increasing
	 *         order.
	 */
	public Vector3f getI0() {
		return inertia;
	}

	@Internal
	public void applyImpulse(Vector3f N, Vector3f RxN, float impulse) {
		float effect = impulse * inv_mass;
		velocity.x += N.x * effect;
		velocity.y += N.y * effect;
		velocity.z += N.z * effect;

		float dwx = inv_Iws.m00 * RxN.x + inv_Iws.m10 * RxN.y + inv_Iws.m20 * RxN.z;
		float dwy = inv_Iws.m01 * RxN.x + inv_Iws.m11 * RxN.y + inv_Iws.m21 * RxN.z;
		float dwz = inv_Iws.m02 * RxN.x + inv_Iws.m12 * RxN.y + inv_Iws.m22 * RxN.z;
		angularVelocity.x += dwx * effect;
		angularVelocity.y += dwy * effect;
		angularVelocity.z += dwz * effect;
	}

	@Internal
	public void applyImpulse(Vector3f impulse, Vector3f R) {
		float dvx = impulse.x * inv_mass;
		float dvy = impulse.y * inv_mass;
		float dvz = impulse.z * inv_mass;
		velocity.x += dvx;
		velocity.y += dvy;
		velocity.z += dvz;

		float Tx = R.y * dvz - R.z * dvy;
		float Ty = R.z * dvx - R.x * dvz;
		float Tz = R.x * dvy - R.y * dvx;

		float dwx = inv_Iws.m00 * Tx + inv_Iws.m10 * Ty + inv_Iws.m20 * Tz;
		float dwy = inv_Iws.m01 * Tx + inv_Iws.m11 * Ty + inv_Iws.m21 * Tz;
		float dwz = inv_Iws.m02 * Tx + inv_Iws.m12 * Ty + inv_Iws.m22 * Tz;
		angularVelocity.x += dwx;
		angularVelocity.y += dwy;
		angularVelocity.z += dwz;
	}

	@Internal
	public void applyImpulseLinear(Vector3f impulse) {
		velocity.x += impulse.x * inv_mass;
		velocity.y += impulse.y * inv_mass;
		velocity.z += impulse.z * inv_mass;
	}

	@Internal
	public void applyImpulseTorque(Vector3f torque) {
		float dwx = inv_Iws.m00 * torque.x + inv_Iws.m10 * torque.y + inv_Iws.m20 * torque.z;
		float dwy = inv_Iws.m01 * torque.x + inv_Iws.m11 * torque.y + inv_Iws.m21 * torque.z;
		float dwz = inv_Iws.m02 * torque.x + inv_Iws.m12 * torque.y + inv_Iws.m22 * torque.z;
		angularVelocity.x += dwx * inv_mass;
		angularVelocity.y += dwy * inv_mass;
		angularVelocity.z += dwz * inv_mass;
	}

	@Internal
	public void applyImpulseTorque(Vector3f axis, float torque) {
		float effect = inv_mass * torque;
		float dwx = inv_Iws.m00 * axis.x + inv_Iws.m10 * axis.y + inv_Iws.m20 * axis.z;
		float dwy = inv_Iws.m01 * axis.x + inv_Iws.m11 * axis.y + inv_Iws.m21 * axis.z;
		float dwz = inv_Iws.m02 * axis.x + inv_Iws.m12 * axis.y + inv_Iws.m22 * axis.z;
		angularVelocity.x += dwx * effect;
		angularVelocity.y += dwy * effect;
		angularVelocity.z += dwz * effect;
	}

	@Internal
	public void applyPseudoImpulse(Vector3f N, Vector3f RxN, float impulse) {
		float effect = impulse * inv_mass;
		pseudoVel.x += N.x * effect;
		pseudoVel.y += N.y * effect;
		pseudoVel.z += N.z * effect;

		float dwx = inv_Iws.m00 * RxN.x + inv_Iws.m10 * RxN.y + inv_Iws.m20 * RxN.z;
		float dwy = inv_Iws.m01 * RxN.x + inv_Iws.m11 * RxN.y + inv_Iws.m21 * RxN.z;
		float dwz = inv_Iws.m02 * RxN.x + inv_Iws.m12 * RxN.y + inv_Iws.m22 * RxN.z;
		pseudoAngVel.x += dwx * effect;
		pseudoAngVel.y += dwy * effect;
		pseudoAngVel.z += dwz * effect;
	}

	@Internal
	public void applyPseudoImpulse(Vector3f impulse, Vector3f R) {
		float dvx = impulse.x * inv_mass;
		float dvy = impulse.y * inv_mass;
		float dvz = impulse.z * inv_mass;
		pseudoVel.x += dvx;
		pseudoVel.y += dvy;
		pseudoVel.z += dvz;

		float Tx = R.y * dvz - R.z * dvy;
		float Ty = R.z * dvx - R.x * dvz;
		float Tz = R.x * dvy - R.y * dvx;

		float dwx = inv_Iws.m00 * Tx + inv_Iws.m10 * Ty + inv_Iws.m20 * Tz;
		float dwy = inv_Iws.m01 * Tx + inv_Iws.m11 * Ty + inv_Iws.m21 * Tz;
		float dwz = inv_Iws.m02 * Tx + inv_Iws.m12 * Ty + inv_Iws.m22 * Tz;
		pseudoAngVel.x += dwx;
		pseudoAngVel.y += dwy;
		pseudoAngVel.z += dwz;
	}

	@Internal
	public void applyPseudoImpulseLinear(Vector3f impulse) {
		pseudoVel.x += impulse.x * inv_mass;
		pseudoVel.y += impulse.y * inv_mass;
		pseudoVel.z += impulse.z * inv_mass;
	}

	@Internal
	public void applyPseudoImpulseTorque(Vector3f torque) {
		float dwx = inv_Iws.m00 * torque.x + inv_Iws.m10 * torque.y + inv_Iws.m20 * torque.z;
		float dwy = inv_Iws.m01 * torque.x + inv_Iws.m11 * torque.y + inv_Iws.m21 * torque.z;
		float dwz = inv_Iws.m02 * torque.x + inv_Iws.m12 * torque.y + inv_Iws.m22 * torque.z;
		pseudoAngVel.x += dwx * inv_mass;
		pseudoAngVel.y += dwy * inv_mass;
		pseudoAngVel.z += dwz * inv_mass;
	}

	@Internal
	public void applyPseudoImpulseTorque(Vector3f axis, float torque) {
		float effect = inv_mass * torque;
		float dwx = inv_Iws.m00 * axis.x + inv_Iws.m10 * axis.y + inv_Iws.m20 * axis.z;
		float dwy = inv_Iws.m01 * axis.x + inv_Iws.m11 * axis.y + inv_Iws.m21 * axis.z;
		float dwz = inv_Iws.m02 * axis.x + inv_Iws.m12 * axis.y + inv_Iws.m22 * axis.z;
		pseudoAngVel.x += dwx * effect;
		pseudoAngVel.y += dwy * effect;
		pseudoAngVel.z += dwz * effect;
	}

	/**
	 * Computes the kinetic energy of the object.
	 * 
	 * @return the kinetic energy
	 */
	public float computeKE() {
		return inv_mass == 0 ? Float.POSITIVE_INFINITY
				: 0.5f / inv_mass * (velocity.lengthSquared() + Vector3f.dot(angularVelocity,
						Matrix3f.transform(Matrix3f.invert(inv_Iws, null), angularVelocity, null)));
	}

	@Override
	public String toString() {
		String string = "RigidBody " + getID();
		string += "\nP = " + getPosition();
		string += "\nV = " + velocity;
		string += "\nW = " + angularVelocity;
		return string;
	}

	/**
	 * Converts the coordinates of a vertex in body-space to a coordinate in
	 * world-space.
	 * 
	 * @param pos  the body-space coordinate
	 * @param dest the world-space coordinate
	 */
	public void vertexToWorldSpace(Vector3f pos, Vector3f dest) {
		bodyToWorld.transformVertex(pos, dest);
	}

	/**
	 * Converts the coordinates of a vertex in world-space to a coordinate in
	 * body-space.
	 * 
	 * @param pos  the world-space coordinate
	 * @param dest the body-space coordinate
	 */
	public void vertexToBodySpace(Vector3f pos, Vector3f dest) {
		bodyToWorld.invertTransformVertex(pos, dest);
	}

	/**
	 * Converts the coordinates of a vector in body-space to a coordinate in
	 * world-space.
	 * 
	 * @param normal the body-space normal
	 * @param dest   the world-space normal
	 */
	public void normalToWorldSpace(Vector3f normal, Vector3f dest) {
		bodyToWorld.transformVector(normal, dest);
	}

	/**
	 * Converts the coordinates of a vector in world-space to a coordinate in
	 * body-space.
	 * 
	 * @param normal the world-space normal
	 * @param dest   the body-space normal
	 */
	public void normalToBodySpace(Vector3f normal, Vector3f dest) {
		bodyToWorld.invertTransformVector(normal, dest);
	}

	/**
	 * @return A list of the {@link Wrapper}s of this object. The list must not be
	 *         modified.
	 */
	@ReadOnly
	public ArrayList<Wrapper> getWrappers() {
		return wrappers;
	}

	protected Wrapper addWrapper(WrapperBuilder builder, long ID) {
		Wrapper wrapper = builder.build(this, ID);
		wrappers.add(wrapper);
		return wrapper;
	}

	protected Wrapper removeWrapper(long ID) {
		for (Wrapper wrapper : wrappers) {
			if (wrapper.getID() == ID) {
				return wrapper;
			}
		}
		return null;
	}

	protected void addAnchorPoint(AnchorPoint point) {
		anchorPoints.add(point);
		bodyToWorld.transformVertex(point.getBodySpacePosition(), point.getWorldSpacePosition());
	}

	protected boolean removeAnchorPoint(AnchorPoint point) {
		return anchorPoints.remove(point);
	}

	/**
	 * @return A list of the anchor points of the constraints attached to that
	 *         object. The list must not be modified.
	 */
	@ReadOnly
	public ArrayList<AnchorPoint> getAnchorPoints() {
		return anchorPoints;
	}

	@Internal
	public void fill(RigidBodyRepr b) {
		b.ID = this.getID();
		b.bodyToWorld.loadFrom(bodyToWorld);
		b.barycentricToWorld.loadFrom(barycentricToWorld);
		b.velocity.set(velocity);
		b.angularVelocity.set(angularVelocity);
		b.contactProperties.set(contactProperties);
		b.inv_mass = inv_mass;
		b.inertia.set(inertia);
		b.inv_Iws.load(inv_Iws);

		b.wrappers.rewind();
		for (Wrapper w : wrappers) {
			w.fill(b.wrappers.getNext());
		}

		// final ReadWriteList<AnchorPoint> anchorPoints; //don't save anchor
		// points for now

		b.mask = mask;
		b.category = category;
		b.flags = flags;
	}

	RigidBody(DefaultParameters params, IDGenerator bodyGenerator, IDGenerator wrapperGenerator, RigidBodyRepr b) {
		super(bodyGenerator.nextID());
		this.bodyToWorld = new Transform(b.bodyToWorld);
		this.barycentricToWorld.loadFrom(b.barycentricToWorld);
		this.velocity.set(b.velocity);
		this.angularVelocity.set(b.angularVelocity);
		this.contactProperties = new ContactProperties(b.contactProperties);
		this.inv_mass = b.inv_mass;
		this.inertia.set(b.inertia);
		Matrix3f.load(b.inv_Iws, inv_Iws);
		this.wrappers = new ArrayList<Wrapper>(b.wrappers.getElementCount());

		for (WrapperRepr w : b.wrappers) {
			this.wrappers.add(w.build(this, wrapperGenerator.nextID()));
		}

		this.anchorPoints = new ArrayList<AnchorPoint>();
		// not filled yet

		this.mask = b.mask;
		this.category = b.category;
		this.sleeping = false;
		this.sleepCounter = 0;
		this.flags = b.flags;

		updateTransforms();

		for (Wrapper wrapper : wrappers) {
			wrapper.placeBox(params.getPadding());
		}
	}

	@Internal
	public void fill(RigidBodyState state) {
		state.ID = this.getID();
		state.barycentricToWorld.loadFrom(barycentricToWorld);
		state.bodyToWorld.loadFrom(bodyToWorld);
		state.velocity.set(getVelocity());
		state.angularVelocity.set(getAngularVelocity());
	}

	/**
	 * @return true if the mass of this body is infinite. A body with infinite mass
	 *         cannot collide with meshes and isn't affected by collisions with
	 *         other bodies.
	 */
	public boolean isKinematic() {
		return getInvMass() == 0;
	}

}
