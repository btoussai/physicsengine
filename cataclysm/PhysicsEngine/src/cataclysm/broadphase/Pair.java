package cataclysm.broadphase;

import cataclysm.contact_creation.DoubleBodyContact;
import cataclysm.datastructures.Identifier;
import cataclysm.wrappers.Wrapper;

/**
 * Repr�sente une paire de {@link Wrapper} en intersection.
 * 
 * @author Briac
 *
 */
public class Pair extends Identifier {

	/**
	 * La premi�re box de la paire, ayant l'ID le plus petit.
	 */
	private final Wrapper wrapperA;

	/**
	 * La seconde box de la paire, ayant l'ID le plus grand.
	 */
	private final Wrapper wrapperB;

	/**
	 * La zone de contact entre les deux objets.
	 */
	private DoubleBodyContact contact;

	Pair(int ID) {
		super(ID);
		this.wrapperA = null;
		this.wrapperB = null;
	}

	public Pair(Wrapper box1, Wrapper box2) {
		super(buildID(box1.getID(), box2.getID()));
		if (box1.compareTo(box2) > 0) {
			this.wrapperA = box2;
			this.wrapperB = box1;
		} else {
			this.wrapperA = box1;
			this.wrapperB = box2;
		}
	}

	void validate(DoubleBodyContact contact) {
		if (contact == null) {
			this.contact = new DoubleBodyContact(wrapperA.getType().maxContacts, wrapperA.getBody(),
					wrapperB.getBody());
		} else {
			this.contact = contact;
			this.contact.refresh(wrapperA.getBody(), wrapperB.getBody());
		}
		this.wrapperA.addCollision();
		this.wrapperB.addCollision();
	}

	public static long buildID(long a, long b) {
		return a < b ? a << 32 | b : b << 32 | a;
	}

	public DoubleBodyContact getContact() {
		return contact;
	}

	public Wrapper getWrapperA() {
		return wrapperA;
	}

	public Wrapper getWrapperB() {
		return wrapperB;
	}

	/**
	 * @return true si les deux rigidbody de la paire sont consid�r�s commme au
	 *         repos.
	 */
	public boolean isSleeping() {
		return wrapperA.getBody().isSleeping() && wrapperB.getBody().isSleeping();
	}

	@Override
	public String toString() {
		return "Pair[" + wrapperA.getID() + " | " + wrapperB.getID() + "]";
	}

}
