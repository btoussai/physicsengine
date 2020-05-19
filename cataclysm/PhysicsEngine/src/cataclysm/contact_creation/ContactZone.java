package cataclysm.contact_creation;

import math.vector.Vector3f;

/**
 * Some basic data about the contact zone.
 * 
 * @author Briac
 *
 */
public class ContactZone {

	public enum FloatLayout {
		Normal, ContactPoints, PenetrationDepths;
	}

	/**
	 * The number of contact points during the previous frame, 0 if there was no
	 * collision
	 */
	private int previousContactCount = 0;

	/**
	 * The number of contact points, 0 if there is no collision
	 */
	private int contactCount = 0;

	/**
	 * The separation distance, < 0 when a collision occurs
	 */
	private float penetrationDepth = Float.NaN;

	/**
	 * The data packed in an array
	 */
	private final float[] floatData;

	/**
	 * What part of object A is colliding
	 */
	private final ContactFeature featureOnA = new ContactFeature();

	/**
	 * What part of object B is colliding
	 */
	private final ContactFeature featureOnB = new ContactFeature();

	/**
	 * Builds a contact zone.
	 * 
	 * @param maxContacts The max number of contact points possible for that contact
	 *                    zone.
	 */
	public ContactZone(int maxContacts) {
		floatData = new float[3 + maxContacts * 4];
	}

	public void getNormal(Vector3f dest) {
		dest.set(floatData[0], floatData[1], floatData[2]);
	}

	/**
	 * @return The separation distance, < 0 if a collision occurs
	 */
	public float getPenetrationDepth() {
		return penetrationDepth;
	}

	/**
	 * @return The number of contact points, 0 if there is no collision
	 */
	public int getContactCount() {
		return contactCount;
	}

	public void getContactPoint(int i, Vector3f dest) {
		dest.x = floatData[3 + 4 * i + 0];
		dest.y = floatData[3 + 4 * i + 1];
		dest.z = floatData[3 + 4 * i + 2];
	}

	public float getContactPointX(int i) {
		return floatData[3 + 4 * i + 0];
	}

	public float getContactPointY(int i) {
		return floatData[3 + 4 * i + 1];
	}

	public float getContactPointZ(int i) {
		return floatData[3 + 4 * i + 2];
	}

	public float getPenetrationDepth(int i) {
		return floatData[3 + 4 * i + 3];
	}

	public void setContactPointAndPenetrationDepth(int i, float x, float y, float z, float penetration) {
		floatData[3 + 4 * i + 0] = x;
		floatData[3 + 4 * i + 1] = y;
		floatData[3 + 4 * i + 2] = z;
		floatData[3 + 4 * i + 3] = penetration;
	}

	public void setContactPoint(int i, Vector3f v) {
		floatData[3 + 4 * i + 0] = v.x;
		floatData[3 + 4 * i + 1] = v.y;
		floatData[3 + 4 * i + 2] = v.z;
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
	 * Indicates there is no collision during that frame
	 */
	void setNoCollision() {
		this.previousContactCount = this.contactCount;
		contactCount = 0;
	}

	/**
	 * Builds a contact zone between two bodies.
	 * 
	 * @param normal           The contact normal, pointing towards body B.
	 * @param penetrationDepth The separation distance
	 * @param contactCount     The number of contact points
	 * @param onA
	 * @param onB
	 */
	public void rebuild(Vector3f normal, float penetrationDepth, int contactCount, ContactFeature onA,
			ContactFeature onB) {
		floatData[0] = normal.x;
		floatData[1] = normal.y;
		floatData[2] = normal.z;
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
		string += "\nNormal: vec3(" + floatData[0] + ", " + floatData[1] + ", " + floatData[2] + ")";
		string += "\nPenetration: " + penetrationDepth;
		for (int i = 0; i < contactCount; i++) {
			string += "\nContact " + i + ": vec3(" + floatData[3 + 4 * i + 0] + ", " + floatData[3 + 4 * i + 1] + ", "
					+ floatData[3 + 4 * i + 2] + ") depth: " + floatData[3 + 4 * i + 3];
		}
		return string;
	}

	public boolean wasCollisionOccuring() {
		return previousContactCount != 0;
	}

	public boolean hasChanged() {
		return contactCount != previousContactCount;
	}

	public int getMaxContacts() {
		return (floatData.length - 3) / 4;
	}

	public void toContactPoint(int i, Vector3f position, Vector3f dest) {
		dest.x = floatData[3 + 4 * i + 0] - position.x;
		dest.y = floatData[3 + 4 * i + 1] - position.y;
		dest.z = floatData[3 + 4 * i + 2] - position.z;
	}
}
