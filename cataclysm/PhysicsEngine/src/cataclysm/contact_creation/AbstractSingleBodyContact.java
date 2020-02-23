package cataclysm.contact_creation;

import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.wrappers.Wrapper;

public abstract class AbstractSingleBodyContact extends AbstractContact {

	protected Wrapper wrapper;
	protected Triangle triangle;

	public AbstractSingleBodyContact(int maxContacts, Wrapper wrapper, Triangle triangle) {
		super(maxContacts);
		this.wrapper = wrapper;
		this.triangle = triangle;
	}

	/**
	 * Cette fonction est utilisée pour réassigner ce contact à un nouveau wrapper
	 * en collision avec un triangle.
	 * 
	 * @param wrapper
	 * @param triangle
	 */
	public void refresh(Wrapper wrapper, Triangle triangle) {
		this.wrapper = wrapper;
		this.triangle = triangle;
		super.getContactArea().resetState();
		resetImpulses();
	}

	/**
	 * Réinitialise les impulsions à zéro. Cette méthode est utile lorsque le Warm
	 * starting est activé.
	 */
	public abstract void resetImpulses();

	public Triangle getTriangle() {
		return triangle;
	}

	public Wrapper getWrapper() {
		return wrapper;
	}

}
