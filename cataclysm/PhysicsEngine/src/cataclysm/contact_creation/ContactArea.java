package cataclysm.contact_creation;

import math.vector.Vector3f;

/**
 * Représente une zone de contact.
 * 
 * @author Briac
 *
 */
public class ContactArea {
	
	private int previousContactCount = 0;

	/**
	 * Le nombre de point de contact, 0 indique qu'il n'y a pas de collision.
	 */
	private int contactCount = 0;

	/**
	 * La normale du contact.
	 */
	private final Vector3f normal = new Vector3f();

	/**
	 * La distance de séparation, négative en cas de collision.
	 */
	private float penetrationDepth = Float.NaN;

	/**
	 * Les points définissant la surface de contact.
	 */
	final Vector3f[] contactPoints;

	/**
	 * Les distances d'intersections individuelles de chaque point de contact.
	 */
	final float[] penetrations;

	/**
	 * La partie de l'objet A en collision.
	 */
	private final ContactFeature featureOnA = new ContactFeature();

	/**
	 * La partie de l'objet B en collision.
	 */
	private final ContactFeature featureOnB = new ContactFeature();

	/**
	 * Construit un contact.
	 * 
	 * @param maxContacts Le nombre maximal de points de contact que ce contact peut
	 *                    générer.
	 */
	public ContactArea(int maxContacts) {
		contactPoints = new Vector3f[maxContacts];
		penetrations = new float[maxContacts];

		for (int i = 0; i < contactPoints.length; i++) {
			contactPoints[i] = new Vector3f();
		}
	}

	public Vector3f getNormal() {
		return normal;
	}

	/**
	 * @return La distance de séparation, négative en cas de collision.
	 */
	public float getPenetrationDepth() {
		return penetrationDepth;
	}

	/**
	 * @return Le nombre de point de contact, 0 indique qu'il n'y a pas de collision.
	 */
	public int getContactCount() {
		return contactCount;
	}

	public Vector3f[] getContactPoints() {
		return contactPoints;
	}

	public float[] getPenetrations() {
		return penetrations;
	}

	public ContactFeature getFeatureOnA() {
		return featureOnA;
	}

	public ContactFeature getFeatureOnB() {
		return featureOnB;
	}

	public boolean isCollisionOccuring() {
		return contactCount != 0;
	}

	/**
	 * Indique qu'il n'y a pas de collision.
	 */
	void setNoCollision() {
		this.previousContactCount = this.contactCount;
		contactCount = 0;
	}

	/**
	 * Construit une zone de contact entre deux solides.
	 * 
	 * @param normal           La normale du contact, dirigée du premier solide vers
	 *                         le deuxième. Le vecteur sera recopié.
	 * @param penetrationDepth La distance de séparation, négative en cas de
	 *                         contact.
	 * @param contactCount     Le nombre de points de contact.
	 * @param onA
	 * @param onB
	 */
	public void rebuild(Vector3f normal, float penetrationDepth, int contactCount, ContactFeature onA,
			ContactFeature onB) {
		this.normal.set(normal);
		this.penetrationDepth = penetrationDepth;
		this.previousContactCount = this.contactCount;
		this.contactCount = contactCount;
		this.featureOnA.setFrom(onA);
		this.featureOnB.setFrom(onB);
	}

	void resetState() {
		this.contactCount = 0;
		this.penetrationDepth = Float.NaN;
	}

	@Override
	public String toString() {
		String string = this.getClass().getSimpleName();
		string += "\nNormal: " + normal;
		string += "\nPenetration: " + penetrationDepth;
		for (Vector3f contact : contactPoints) {
			string += "\nContact: " + contact;
		}
		return string;
	}

	public boolean wasCollisionOccuring() {
		return previousContactCount != 0;
	}
	
	public boolean hasChanged() {
		return contactCount != previousContactCount;
	}

}
