package cataclysm.contact_creation;

import org.lwjgl.util.vector.Matrix3f;
import org.lwjgl.util.vector.Vector3f;

/**
 * Représente un contact entre deux solides. Les solides ne se touchent pas
 * forcément, le contact n'est alors pas actif.
 * 
 * @author Briac
 *
 */
public abstract class AbstractContact {

	protected final ContactArea area;

	protected float friction = 0;
	protected float elasticity = 0;
	protected float stickiness = 0;

	public AbstractContact(int maxContacts) {
		area = new ContactArea(maxContacts);
	}

	protected void mixContactProperties(ContactProperties A, ContactProperties B) {
		this.friction = 0.5f * (A.getFriction() + B.getFriction());
		this.elasticity = 0.5f * (A.getElasticity() + B.getElasticity());
		this.stickiness = 0.5f * (A.getStickiness() + B.getStickiness());
	}
	
	public abstract void velocityStart();

	public abstract void solveVelocity();
	
	public abstract void positionStart(float timeStep);

	public abstract void solvePosition();
	

	public ContactArea getContactArea() {
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
		return area.contactPoints.length;
	}

}
