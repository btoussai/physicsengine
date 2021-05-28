package cataclysm.contact_creation;

import java.util.ArrayList;
import java.util.List;

import cataclysm.wrappers.Wrapper;

public abstract class AbstractDoubleBodyContact extends AbstractContact {

	protected Wrapper wrapperA;
	protected Wrapper wrapperB;
	
	protected List<Wrapper> previousOwners = new ArrayList<>();

	/**
	 * Permet de déterminer si les deux wrappers ont été mis à jour avant de mettre
	 * à jour le contact.
	 */
	private boolean updateFlag = false;

	/**
	 * Builds an abstract contact with a contact zone having at most maxContacts
	 * contact points.
	 * 
	 * @param maxContacts
	 * @param wrapperA
	 * @param wrapperB
	 */
	public AbstractDoubleBodyContact(int maxContacts, Wrapper wrapperA, Wrapper wrapperB) {
		super(maxContacts);
		this.wrapperA = wrapperA;
		this.wrapperB = wrapperB;
	}

	/**
	 * Builds an abstract contact without a contact zone. Reserved for internal use.
	 * 
	 * @param wrapperA
	 * @param wrapperB
	 */
	protected AbstractDoubleBodyContact(Wrapper wrapperA, Wrapper wrapperB) {
		super();
		this.wrapperA = wrapperA;
		this.wrapperB = wrapperB;
	}

	/**
	 * Cette fonction est utilisée pour réassigner ce contact à un nouveau couple de
	 * wrappers en collision.
	 * 
	 * @param wrapperA
	 * @param wrapperB
	 */
	public void refresh(Wrapper wrapperA, Wrapper wrapperB) {
		previousOwners.add(this.wrapperA);
		previousOwners.add(this.wrapperB);
		
		this.wrapperA = wrapperA;
		this.wrapperB = wrapperB;
		if (super.getContactArea() != null) {// we have to check in case this is a ContactProxy
			super.getContactArea().resetState();
			resetImpulses();
		}
	}

	public Wrapper getWrapperA() {
		return wrapperA;
	}

	public Wrapper getWrapperB() {
		return wrapperB;
	}

	public boolean getUpdateFlagAndFlip() {
		boolean prev = updateFlag;
		updateFlag = !prev;
		return prev;
	}

	public void wakeUp() {
		this.wrapperA.getBody().setSleeping(false);
		this.wrapperA.getBody().setSleepCounter(0);
		this.wrapperB.getBody().setSleeping(false);
		this.wrapperB.getBody().setSleepCounter(0);
	}

	public Wrapper getOther(Wrapper w) {
		if (w != wrapperA && w != wrapperB) {
			throw new IllegalArgumentException("Argument must be in the pair of wrappers of this contact");
		}
		return w == wrapperA ? wrapperB : wrapperA;
	}

	@Override
	public String toString() {
		return this.getClass().getSimpleName() + ": " + wrapperA + " | " + wrapperB;
	}

}
