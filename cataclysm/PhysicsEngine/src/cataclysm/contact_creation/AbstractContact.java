package cataclysm.contact_creation;

import math.vector.Matrix3f;
import math.vector.Vector3f;

/**
 * Represents a contact between two objects. If they don't collide, the contact
 * is considered to be 'inactive'
 * 
 * @author Briac
 *
 */
public abstract class AbstractContact {

	protected final ContactZone area;

	protected float friction = 0;
	protected float elasticity = 0;

	public AbstractContact(int maxContacts) {
		area = new ContactZone(maxContacts);
	}
	
	/**
	 * Builds a contact without a contact zone. Reserved for internal use.
	 */
	AbstractContact() {
		area = null;
	}

	protected void mixContactProperties(ContactProperties A, ContactProperties B) {
		this.friction = 0.5f * (A.getFriction() + B.getFriction());
		this.elasticity = 0.5f * (A.getElasticity() + B.getElasticity());
	}

	public abstract void velocityStart();

	public abstract void warmStart();

	public abstract void resetImpulses();

	public abstract void solveVelocity();

	public abstract void positionStart(float timeStep);

	public abstract void solvePosition();

	public ContactZone getContactArea() {
		return area;
	}

	protected Vector3f[] initArray(int length) {
		Vector3f[] array = new Vector3f[length];
		for (int i = 0; i < array.length; i++) {
			array[i] = new Vector3f();
		}
		return array;
	}

	protected Matrix3f[] initMatrixArray(int length) {
		Matrix3f[] array = new Matrix3f[length];
		for (int i = 0; i < array.length; i++) {
			array[i] = new Matrix3f();
		}
		return array;
	}

	public int getMaxContacts() {
		return area.getMaxContacts();
	}

}
