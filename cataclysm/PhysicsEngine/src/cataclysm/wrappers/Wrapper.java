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
 * Représente une enveloppe générale pour un objet convexe. Un objet peut
 * posséder plusieurs enveloppes.
 * 
 * @see SphereWrapper
 * @see CapsuleWrapper
 * @see ConvexHullWrapper
 * @author Briac
 *
 */
public abstract class Wrapper extends Identifier implements Comparable<Wrapper> {

	/**
	 * Représente le type de l'enveloppe.
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
	 * La transformation wrapper-space --> body-space.
	 */
	protected final Transform wrapperToBody;

	/**
	 * Le centre de masse de l'enveloppe, en wrapper-space et en world-space.
	 */
	private final TransformableVec3 centroid;

	/**
	 * Le rigidbody possédant cette enveloppe.
	 */
	private final RigidBody body;

	/**
	 * Une simple AABB centrée sur l'objet. Permet de détecter les paires d'objet en
	 * collision.
	 */
	private final BroadPhaseNode<Wrapper> node;

	/**
	 * Le rayon maximal de l'enveloppe, i.e. la taille de la {@link WrapperBox}.
	 */
	private float maxRadius;

	/**
	 * La masse, le volume, etc...
	 */
	protected final MassProperties massProperties;

	/**
	 * La liste des contacts entre ce wrapper et des triangles appartenant à des
	 * staticmesh.
	 */
	protected final ArrayList<AbstractSingleBodyContact> meshContacts = new ArrayList<AbstractSingleBodyContact>(1);

	/**
	 * La liste des contacts entre ce wrapper et d'autres wrappers à proximité.
	 */
	protected final ArrayList<AbstractDoubleBodyContact> bodyContacts = new ArrayList<AbstractDoubleBodyContact>(1);

	/**
	 * Construit une nouvelle enveloppe de collision.
	 * 
	 * @param body           Le corps rigide possédant cette enveloppe.
	 * @param wrapperToBody  La transformation indiquant la position et
	 *                       l'orientation de l'enveloppe dans le rep�re du corps
	 *                       rigide.
	 * @param massProperties Les propriétés massiques du builder.
	 * @param maxRadius      Le rayon maximal de l'enveloppe, c'est à dire la
	 *                       distance maximale entre un point de l'enveloppe et son
	 *                       centre géométrique.
	 * @param ID             Un identifiant unique pour ce wrapper.
	 */
	protected Wrapper(RigidBody body, Transform wrapperToBody, MassProperties massProperties, float maxRadius,
			long ID) {
		super(ID);
		this.centroid = new TransformableVec3();
		this.wrapperToBody = new Transform(wrapperToBody);
		this.body = body;
		this.maxRadius = maxRadius;
		this.node = new BroadPhaseNode<Wrapper>(new AABB(), this);
		this.massProperties = new MassProperties(massProperties);
	}

	/**
	 * Ce constructeur ne doit être utilisé que par {@link TriangleAsHull}
	 */
	protected Wrapper() {
		super(-1);
		this.centroid = new TransformableVec3();
		this.body = null;
		this.maxRadius = 0;
		this.node = null;
		this.wrapperToBody = null;
		this.massProperties = null;
	}

	/**
	 * Change la position du centre de masse de l'enveloppe (en wrapper-space). <br>
	 * Cette fonction est appel�e par
	 * {@link ConvexHullWrapper#computeInertia(Vector3f, Matrix3f, PolyhedralMassProperties)}
	 * car la position du centre de masse de l'enveloppe n'est pas connue à
	 * l'avance.
	 * 
	 * @param centroid le centre de masse de l'enveloppe en wrapper-space.
	 */
	protected void placeCentroid(Vector3f centroid) {
		this.centroid.getInputSpaceCoord().set(centroid);
	}

	/**
	 * Cette méthode permet de recentrer l'{@link AABB} autour du {@link Wrapper}.
	 * 
	 * @param padding Une marge additionnelle autour de l'AABB.
	 */
	protected void placeBox(float padding) {
		Vector3f centroid = this.getCentroid();
		float halfSize = maxRadius + padding;
		node.getBox().set(centroid, halfSize);
	}

	/**
	 * @return Le corps rigide possédant cet objet.
	 */
	public RigidBody getBody() {
		return body;
	}

	public BroadPhaseNode<Wrapper> getNode() {
		return node;
	}

	/**
	 * @return Le centre de masse de l'enveloppe en world-space
	 */
	public Vector3f getCentroid() {
		return centroid.getOutputSpaceCoord();
	}

	/**
	 * @return Le centre de masse de l'enveloppe en wrapper-space
	 */
	public Vector3f getCentroidWrapperSpace() {
		return centroid.getInputSpaceCoord();
	}

	/**
	 * @return Le rayon maximal de cette enveloppe de collision.
	 */
	public float getMaxRadius() {
		return maxRadius;
	}

	/**
	 * @return La liste des contacts entre cette enveloppe et des triangles.
	 */
	public ArrayList<AbstractSingleBodyContact> getMeshContacts() {
		return meshContacts;
	}

	/**
	 * @return La liste des contacts entre ce wrapper et d'autres wrappers à
	 *         proximité.
	 */
	public ArrayList<AbstractDoubleBodyContact> getBodyContacts() {
		return bodyContacts;
	}

	@Override
	public int compareTo(Wrapper other) {
		return this.getType().maxContacts - other.getType().maxContacts;
	}

	/**
	 * Recalcule la position des sommets de l'enveloppe en world-space.
	 * 
	 * @param wrapperToWorld La transformation wrapper-space --> world-space.
	 */
	public void transform(Transform wrapperToWorld) {
		this.centroid.transformAsVertex(wrapperToWorld);
	}

	/**
	 * @return La transformation wrapper-space --> body-space.
	 */
	public Transform getTransform() {
		return wrapperToBody;
	}

	/**
	 * @return le type de l'enveloppe de collision.
	 */
	public abstract Type getType();

	/**
	 * Calcule la position du point le plus éloigné dans la direction indiquée.
	 * 
	 * @param direction Le vecteur direction, pas nécessairement unitaire.
	 * @param negate    indique s'il faut chercher à l'opposé de direction.
	 * @param dest      Le vecteur dans lequel stocker le résultat.
	 */
	public abstract void getSupport(Vector3f direction, boolean negate, Vector3f dest);

	/**
	 * Applique un facteur d'echelle sur l'enveloppe. Le changement d'échelle est
	 * appliqué à partir du centre de masse de l'enveloppe.
	 * 
	 * @param scaleFactor Le facteur d'échelle.
	 */
	public void scale(float scaleFactor) {
		this.massProperties.scale(scaleFactor);
		float padding = 0.5f * (node.getBox().minX + node.getBox().maxX) - this.maxRadius;
		this.maxRadius *= scaleFactor;
		placeBox(padding);
	}

	/**
	 * Calcule le tenseur d'inertie et le centre de masse de l'enveloppe en
	 * body-space. Le tenseur d'inertie est exprimé autour de l'origine. Recalcule
	 * également les propriétés massique de l'enveloppe (masse, volume, etc...)
	 * 
	 * @param centerOfMass un vecteur de destination pour le centre de masse.
	 * @param inertia      une matrice de destination pour le tenseur d'inertie.
	 * @param poly         TODO
	 * @return la masse de l'enveloppe.
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
		StringBuilder sb = new StringBuilder("Wrapper " + getType() + " : \n");
		sb.append("\nMeshContacts:" + meshContacts.size());
		sb.append("\nBodyContacts:" + bodyContacts.size());
		return sb.toString();
	}

	public Wrapper(RigidBody body, WrapperRepr w, long ID) {
		super(ID);

		this.centroid = new TransformableVec3(w.centroid);
		this.wrapperToBody = new Transform(w.wrapperToBody);
		this.body = body;
		this.maxRadius = w.maxRadius;
		this.node = new BroadPhaseNode<Wrapper>(new AABB(), this);
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
