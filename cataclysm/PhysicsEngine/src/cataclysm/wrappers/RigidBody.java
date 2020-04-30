package cataclysm.wrappers;

import java.util.ArrayList;

import cataclysm.DefaultCollisionFilter;
import cataclysm.DefaultParameters;
import cataclysm.Epsilons.Sleep;
import cataclysm.constraints.AnchorPoint;
import cataclysm.contact_creation.ContactProperties;
import cataclysm.datastructures.IDGenerator;
import cataclysm.datastructures.Identifier;
import math.MatrixOps;
import math.vector.Matrix3f;
import math.vector.Matrix4f;
import math.vector.Vector3f;

/**
 * Représente un corps rigide.
 * 
 * @author Briac
 *
 */
public class RigidBody extends Identifier {

	/**
	 * La position et la rotation de l'objet en world-space
	 */
	private final Transform bodyToWorld;

	/**
	 * La position et la rotation du centre de masse en world-space. Le tenseur
	 * d'inertie est diagonal dans le repère barycentrique.
	 */
	private final Transform barycentricToWorld = new Transform();

	/**
	 * Le vecteur vitesse du centre de masse en m/s (world-space)
	 */
	private final Vector3f velocity = new Vector3f();

	/**
	 * Le vecteur vitesse angulaire du centre de masse en rad/s (world-space)
	 */
	private final Vector3f angularVelocity = new Vector3f();

	/**
	 * true si l'objet est soumis aux forces extérieures.
	 */
	private boolean gravity = false;

	/**
	 * La friction et l'élasticité de l'objet.
	 */
	private final ContactProperties contactProperties;

	/**
	 * La masse inverse de l'objet: 1.0f / mass.
	 */
	private float inv_mass;

	/**
	 * Les 3 moments principaux d'inertie de l'objet, rangés par ordre croissant et
	 * divisés par la masse totale de l'objet.
	 */
	private final Vector3f inertia = new Vector3f();

	/**
	 * Le tenseur d'inertie inversé, en world-space. <br>
	 * Plus précis�ment:<br>
	 * <br>
	 * inv_Iws = Orientation^T * diag( 1 / inertia ) * Orientation
	 */
	private final Matrix3f inv_Iws = new Matrix3f();

	/**
	 * Les enveloppes utilisées pour les collisions.
	 */
	private final ArrayList<Wrapper> wrappers;

	/**
	 * Les points d'ancrage des contraintes attachées à ce rigidbody.
	 */
	private final ArrayList<AnchorPoint> anchorPoints;

	/**
	 * Un vecteur vitesse ne persistant pas d'une frame à l'autre. Il permet de
	 * corriger les erreurs de position.
	 */
	private final Vector3f pseudoVel = new Vector3f();

	/**
	 * Un vecteur vitesse angulaire ne persistant pas d'une frame à l'autre. Il
	 * permet de corriger les erreurs de rotation.
	 */
	private final Vector3f pseudoAngVel = new Vector3f();

	/**
	 * Permet de créer des masques pour filter les collisions avec les objets de la
	 * catégorie correspondante.
	 */
	private int mask = 0xFFFFFFFF;

	/**
	 * Représente la catégorie de l'objet. Ceci permet de filter les collisions.
	 */
	private int category = 0xFFFFFFFF;

	/**
	 * Indique si le rigidbody est considéré comme totalement immobile.
	 */
	private boolean sleeping = false;

	/**
	 * Le nombre de frame consécutives passées 'immobile'. Lorsque le compteur
	 * dépasse {@link Sleep#FRAMES_SPENT_AT_REST} et que
	 * {@link Sleep#SLEEPING_ALLOWED} vaut true, le rigidbody est placé en sommeil.
	 */
	private int sleepCounter = 0;

	/**
	 * Indique si la rotation pour ce rigidbody doit être bloquée ou non.
	 */
	private boolean rotationBlocked = false;

	/**
	 * Construit un nouveau rigidbody.
	 * 
	 * @param transform La position et la rotation de l'objet en world-space.
	 * @param params    Les param�tres par d�faut.
	 * @param generator Un g�n�rateur d'ID pour g�n�rer les identifiants de ce
	 *                  rigidbody et de ses wrappers.
	 * @param builders  Les enveloppes permettant de calculer les collisions.
	 */
	RigidBody(Matrix4f transform, DefaultParameters params, IDGenerator generator, PolyhedralMassProperties poly,
			WrapperBuilder... builders) {
		super(generator.nextID());
		this.bodyToWorld = new Transform(transform);
		this.wrappers = new ArrayList<Wrapper>(builders.length);
		this.anchorPoints = new ArrayList<AnchorPoint>(0);

		for (int i = 0; i < builders.length; i++) {
			this.wrappers.add(builders[i].build(this, generator.nextID()));
		}

		computeMassProperties(poly);

		updateTransform(new Transform());

		for (Wrapper wrapper : wrappers) {
			wrapper.placeBox(params.getPadding());
		}

		contactProperties = new ContactProperties(params.getContactProperties());
		this.gravity = params.isGravity();
	}

	/**
	 * Met à jour les enveloppes de l'objet avec la position et la rotation actuelle
	 * de l'objet. Met également à jour le tenseur d'inertie et les points d'ancrage
	 * des contraintes liées à ce rigidbody. Cette fonction est appelée
	 * automatiquement lors de la mise à jour de la simulation.
	 * 
	 * @param temp Une variable de travail pour les calculs intermédiaires.
	 */
	public void updateTransform(Transform temp) {
		if (!rotationBlocked) {
			float I1 = 1.0f / inertia.x;
			float I2 = 1.0f / inertia.y;
			float I3 = 1.0f / inertia.z;

			MatrixOps.changeOfBasis(I1, I2, I3, barycentricToWorld.getRotation(), inv_Iws);
		} else {
			inv_Iws.setZero();
		}

		for (Wrapper wrapper : wrappers) {
			Transform.compose(bodyToWorld, wrapper.getTransform(), temp);
			wrapper.transform(temp);
		}

		for (AnchorPoint point : anchorPoints) {
			bodyToWorld.transformVertex(point.getBodySpacePosition(), point.getWorldSpacePosition());
		}

	}

	/**
	 * Recalcule la masse, le centre de masse, le tenseur d'inertie, etc... <br>
	 * Si un des wrappers a subi une modification manuelle après la création du
	 * rigidbody (déplacement / rotation, modification de la masse, ou changement
	 * d'échelle) il faut appeler cette fonction pour mettre à jour l'objet.
	 * L'addition ou la suppression d'une enveloppe modifie également la répartition
	 * des masses, il faut donc également appeler cette fonction dans ce cas.
	 * 
	 * @param poly Permet de calculer les propriétés massiques de l'objet
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
		bodyCenterOfMass.scale(-inv_mass);
		PolyhedralMassProperties.translateInertia(bodyInertia, bodyCenterOfMass, mass);
		bodyCenterOfMass.negate();

		MatrixOps.eigenValues(bodyInertia, inertia, this.barycentricToWorld.getRotation());
		inertia.scale(inv_mass);
		this.barycentricToWorld.getTranslation().set(bodyCenterOfMass);

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
	 * La fonction {@link #updateTransform(Transform)} est automatiquement appelée à
	 * la fin de la fonction pour finir la mise à jour de l'enveloppe.
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
			wrapper.getTransform().getTranslation().scale(scaleFactor);

			Vector3f centroid = wrapper.getCentroidWrapperSpace();
			float dx = centroid.x * (scaleFactor - 1.0f);
			float dy = centroid.y * (scaleFactor - 1.0f);
			float dz = centroid.z * (scaleFactor - 1.0f);
			centroid.translate(dx, dy, dz);

			// In case the wrapper is a hull, we have to translate everything so that the
			// vertices stay at the same relative position with respect to the center of
			// mass.
			if (wrapper instanceof ConvexHullWrapper) {
				((ConvexHullWrapper) wrapper).getData().translate(dx, dy, dz);
			}
		}

		setSleeping(false);
		setSleepCounter(0);

		updateTransform(new Transform());
	}

	public boolean isGravity() {
		return gravity;
	}

	public void setGravity(boolean gravity) {
		this.gravity = gravity;
	}

	/**
	 * Déplace le rigidbody du vecteur translation.
	 * 
	 * @param translation
	 */
	public void translate(Vector3f translation) {
		barycentricToWorld.translate(translation);
		bodyToWorld.translate(translation);
	}

	/**
	 * Déplace le rigidbody du vecteur translation.
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
	 * Fait tourner le rigidbody autour de son centre de masse.
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
	 * Fait tourner le rigidbody autour de son origine.
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
	 * @return La position du centre de masse. i.e. L'origine du repère
	 *         barycentrique.
	 */
	public Vector3f getPosition() {
		return barycentricToWorld.getTranslation();
	}

	/**
	 * @return La rotation du repère barycentrique.
	 */
	public Matrix3f getOrientation() {
		return barycentricToWorld.getRotation();
	}

	/**
	 * @return Le repère barycentrique du rigidbody.
	 */
	public Transform getBarycentricTransform() {
		return barycentricToWorld;
	}

	/**
	 * @return L'origine du rigidbody en world-space, correspond à la position du
	 *         repère body-space.
	 */
	public Vector3f getOriginPosition() {
		return bodyToWorld.getTranslation();
	}

	/**
	 * @return La rotation du repère body-space.
	 */
	public Matrix3f getOriginOrientation() {
		return bodyToWorld.getRotation();
	}

	/**
	 * @return Le repère body-space du rigidbody.
	 */
	public Transform getOriginTransform() {
		return bodyToWorld;
	}

	public Vector3f getVelocity() {
		return velocity;
	}

	public Vector3f getAngularVelocity() {
		return angularVelocity;
	}

	public Vector3f getPseudoVelocity() {
		return pseudoVel;
	}

	public Vector3f getPseudoAngularVelocity() {
		return pseudoAngVel;
	}

	/**
	 * @return la masse inverse de l'objet. ( {@code 1.0f / mass} par d�faut)
	 */
	public float getInvMass() {
		return inv_mass;
	}

	/**
	 * Modifie la masse inverse de ce rigidbody. <br>
	 * Un rigidbody dont la masse inverse est nulle (i.e. sa masse est infinie) ne
	 * peut pas être mis en mouvement par des collisions avec d'autres objets et sa
	 * vitesse sera conservée. Il restera affecté par les forces extérieures mais ne
	 * pourra pas entrer en collision avec le sol ou avec un autre objet dont la
	 * masse inverse est nulle. <br>
	 * Le comportement est indéfini lorsque deux objets avec une masse inverse nulle
	 * sont reliés par une contrainte.
	 * 
	 * @param inv_mass la masse inverse de l'objet. ( {@code 1.0f / mass} par
	 *                 défaut)
	 */
	public void setInvMass(float inv_mass) {
		this.inv_mass = inv_mass;
	}

	/**
	 * @return les propriétés de l'objet lors des contacts (friction, élasticité...)
	 */
	public ContactProperties getContactProperties() {
		return contactProperties;
	}

	/**
	 * @return Le masque du rigidbody. Deux rigidbody peuvent former des contacts
	 *         seulement si <br>
	 *         {@code (maskA & categoryB) != 0 && (maskB & categoryA) != 0} par
	 *         défaut.
	 * @see DefaultCollisionFilter
	 */
	public int getMask() {
		return mask;
	}

	/**
	 * Modifie le masque du rigidbody. Deux rigidbody peuvent former des contacts
	 * seulement si <br>
	 * {@code (maskA & categoryB) != 0 && (maskB & categoryA) != 0} par défaut.
	 * 
	 * @see DefaultCollisionFilter
	 * 
	 * @param mask
	 */
	public void setMask(int mask) {
		this.mask = mask;
	}

	/**
	 * 
	 * @return La catégorie du rigidbody. Deux rigidbody peuvent former des contacts
	 *         seulement si <br>
	 *         {@code (maskA & categoryB) != 0 && (maskB & categoryA) != 0} par
	 *         défaut.
	 * @see DefaultCollisionFilter
	 */
	public int getCategory() {
		return category;
	}

	/**
	 * Modifie la catégorie du rigidbody. Deux rigidbody peuvent former des contacts
	 * seulement si <br>
	 * {@code (maskA & categoryB) != 0 && (maskB & categoryA) != 0} par défaut.
	 * 
	 * @see DefaultCollisionFilter
	 * 
	 * @param category
	 */
	public void setCategory(int category) {
		this.category = category;
	}

	public boolean isSleeping() {
		return sleeping;
	}

	public void setSleeping(boolean sleeping) {
		this.sleeping = sleeping;
	}

	/**
	 * @return Le nombre de frames consécutives durant lesquelles le rigidbody a été
	 *         considéré comme immobile. Lorsque le compteur dépasse
	 *         {@link Sleep#FRAMES_SPENT_AT_REST} et que
	 *         {@link Sleep#SLEEPING_ALLOWED} vaut true, le rigidbody est placé en
	 *         sommeil.
	 */
	public int getSleepCounter() {
		return sleepCounter;
	}

	/**
	 * Modifie le nombre de frames consécutives durant lesquelles le rigidbody a été
	 * considéré comme immobile. Lorsque le compteur dépasse
	 * {@link Sleep#FRAMES_SPENT_AT_REST} et que {@link Sleep#SLEEPING_ALLOWED} vaut
	 * true, le rigidbody est placé en sommeil.
	 * 
	 * @param sleepCounter
	 */
	public void setSleepCounter(int sleepCounter) {
		this.sleepCounter = sleepCounter;
	}

	/**
	 * @return true si la rotation est bloquée pour ce rigidbody. Un rigidbody dont
	 *         la rotation est bloquée aura un tenseur d'inertie infini
	 *         (conceptuellement seulement, les moments d'inertie principaux restent
	 *         inchangés). Cela signifie que sa vitesse angulaire sera conservée,
	 *         peut importe les chocs qu'il subit.
	 */
	public boolean isRotationBlocked() {
		return rotationBlocked;
	}

	/**
	 * Permet de bloquer ou de débloquer la rotation de ce rigidbody. Un rigidbody
	 * dont la rotation est bloquée aura un tenseur d'inertie infini
	 * (conceptuellement seulement, les moments d'inertie principaux restent
	 * inchangés). Cela signifie que sa vitesse angulaire sera conservée, peut
	 * importe les chocs qu'il subit.
	 * 
	 * @param rotationBlocked
	 */
	public void setRotationBlocked(boolean rotationBlocked) {
		this.rotationBlocked = rotationBlocked;
	}

	/**
	 * @return Le tenseur d'inertie en worldSpace, inversé
	 */
	public Matrix3f getInvIws() {
		return inv_Iws;
	}

	/**
	 * @return Les moments d'inertie principaux du tenseur d'inertie.
	 */
	public Vector3f getI0() {
		return inertia;
	}

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

	public void applyImpulseLinear(Vector3f impulse) {
		velocity.x += impulse.x * inv_mass;
		velocity.y += impulse.y * inv_mass;
		velocity.z += impulse.z * inv_mass;
	}

	public void applyImpulseTorque(Vector3f torque) {
		float dwx = inv_Iws.m00 * torque.x + inv_Iws.m10 * torque.y + inv_Iws.m20 * torque.z;
		float dwy = inv_Iws.m01 * torque.x + inv_Iws.m11 * torque.y + inv_Iws.m21 * torque.z;
		float dwz = inv_Iws.m02 * torque.x + inv_Iws.m12 * torque.y + inv_Iws.m22 * torque.z;
		angularVelocity.x += dwx * inv_mass;
		angularVelocity.y += dwy * inv_mass;
		angularVelocity.z += dwz * inv_mass;
	}

	public void applyImpulseTorque(Vector3f axis, float torque) {
		float effect = inv_mass * torque;
		float dwx = inv_Iws.m00 * axis.x + inv_Iws.m10 * axis.y + inv_Iws.m20 * axis.z;
		float dwy = inv_Iws.m01 * axis.x + inv_Iws.m11 * axis.y + inv_Iws.m21 * axis.z;
		float dwz = inv_Iws.m02 * axis.x + inv_Iws.m12 * axis.y + inv_Iws.m22 * axis.z;
		angularVelocity.x += dwx * effect;
		angularVelocity.y += dwy * effect;
		angularVelocity.z += dwz * effect;
	}

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

	public void applyPseudoImpulseLinear(Vector3f impulse) {
		pseudoVel.x += impulse.x * inv_mass;
		pseudoVel.y += impulse.y * inv_mass;
		pseudoVel.z += impulse.z * inv_mass;
	}

	public void applyPseudoImpulseTorque(Vector3f torque) {
		float dwx = inv_Iws.m00 * torque.x + inv_Iws.m10 * torque.y + inv_Iws.m20 * torque.z;
		float dwy = inv_Iws.m01 * torque.x + inv_Iws.m11 * torque.y + inv_Iws.m21 * torque.z;
		float dwz = inv_Iws.m02 * torque.x + inv_Iws.m12 * torque.y + inv_Iws.m22 * torque.z;
		pseudoAngVel.x += dwx * inv_mass;
		pseudoAngVel.y += dwy * inv_mass;
		pseudoAngVel.z += dwz * inv_mass;
	}

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
	 * Calcule l'énergie cinétique de l'objet, utile pour le debug.
	 * 
	 * @return L'énergie cinétique de l'objet.
	 */
	public float computeEc() {
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
	 * Effectue une conversion body-space ---> world-space pour un sommet.
	 * 
	 * @param msPos La coordonnée en body-space
	 * @param wsPos La coordonnées de destination en world-space
	 */
	public void vertexToWorldSpace(Vector3f msPos, Vector3f wsPos) {
		bodyToWorld.transformVertex(msPos, wsPos);
	}

	/**
	 * Effectue une conversion world-space ---> body-space pour un sommet.
	 * 
	 * @param wsPos La coordonnées en world-space
	 * @param msPos La coordonnée de destination en body-space
	 */
	public void vertexToBodySpace(Vector3f wsPos, Vector3f msPos) {
		bodyToWorld.invertTransformVertex(wsPos, msPos);
	}

	/**
	 * Effectue une conversion body-space ---> world-space pour un vecteur. C'est à
	 * dire que seule la rotation est appliquée, la translation est ignorée.
	 * 
	 * @param msNormal Le vecteur en body-space
	 * @param wsNormal Le vecteur de destination en world-space
	 */
	public void normalToWorldSpace(Vector3f msNormal, Vector3f wsNormal) {
		bodyToWorld.transformVector(msNormal, wsNormal);
	}

	/**
	 * Effectue une conversion world-space ---> body-space pour un vecteur. C'est à
	 * dire que seule la rotation est appliquée, la translation est ignorée.
	 * 
	 * @param wsNormal Le vecteur en world-space.
	 * @param msNormal Le vecteur de destination en body-space.
	 */
	public void normalToBodySpace(Vector3f wsNormal, Vector3f msNormal) {
		bodyToWorld.invertTransformVector(wsNormal, msNormal);
	}

	/**
	 * @return La liste des wrappers de ce rigidbody. Cette liste ne doit pas être
	 *         modifiée manuellement.
	 */
	public ArrayList<Wrapper> getWrappers() {
		return wrappers;
	}

	/**
	 * Ajoute un wrapper à ce rigidbody
	 * 
	 * @param builder
	 * @param ID
	 * @return
	 */
	protected Wrapper addWrapper(WrapperBuilder builder, long ID) {
		Wrapper wrapper = builder.build(this, ID);
		wrappers.add(wrapper);
		return wrapper;
	}

	/**
	 * Supprime un wrapper de ce rigidbody
	 * 
	 * @param ID
	 * @return
	 */
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
	 * @return La liste des points d'ancrage sur ce rigidbody. Cette liste ne doit
	 *         pas être modifiée manuellement.
	 */
	public ArrayList<AnchorPoint> getAnchorPoints() {
		return anchorPoints;
	}

}
