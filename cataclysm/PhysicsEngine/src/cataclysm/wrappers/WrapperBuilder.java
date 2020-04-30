package cataclysm.wrappers;

import math.vector.Matrix4f;

/**
 * Cette classe permet d'instancier une enveloppe de collision pour un
 * rigidbody.
 * 
 * @author Briac
 *
 */
public abstract class WrapperBuilder {

	protected final Transform wrapperToBody;
	protected final MassProperties massProperties;

	WrapperBuilder(Matrix4f wrapperToBody) {
		this.wrapperToBody = new Transform(wrapperToBody);
		this.massProperties = new MassProperties(0, 0, false, 1);
	}
	
	public Transform getTransform() {
		return wrapperToBody;
	}
	
	public void setTransform(Matrix4f transform) {
		this.wrapperToBody.loadFrom(transform);
	}

	/**
	 * Les enveloppes construites par ce builder seront vide ou pleines. Ceci
	 * n'influence que le tenseur d'inertie du rigidbody poss�dent l'enveloppe.
	 * 
	 * @param hollow
	 * @return this
	 */
	public WrapperBuilder setHollow(boolean hollow) {
		this.massProperties.setHollow(hollow);
		return this;
	}

	/**
	 * Change la densit� des enveloppes construites par ce builder. La masse des
	 * enveloppes sera d�duite de la densit�. <br>
	 * Si l'enveloppe est pleine, la densit� correspond � la masse surfacique. <br>
	 * Si l'enveloppe est creuse, la densit� correspond � la masse volumique.
	 * 
	 * @param density
	 * @return this
	 */
	public WrapperBuilder setDensity(float density) {
		this.massProperties.setDensity(density);
		this.massProperties.setUseDensity(true);
		return this;
	}

	/**
	 * Change la masse des enveloppes construites par ce builder. La densit� sera
	 * d�duite de la masse. <br>
	 * Si l'enveloppe est pleine, la densit� correspond � la masse surfacique. <br>
	 * Si l'enveloppe est creuse, la densit� correspond � la masse volumique.
	 * 
	 * @param mass
	 */
	public void setMass(float mass) {
		this.massProperties.setMass(mass);
		this.massProperties.setUseDensity(false);
	}

	/**
	 * Construit une nouvelle enveloppe de collision pour le corps rigide pass� en
	 * param�tre.
	 * 
	 * @param body Le corps rigide pour lequel l'enveloppe est construite.
	 * @param ID
	 * @return Une nouvelle enveloppe de collision.
	 */
	abstract Wrapper build(RigidBody body, long ID);

	/**
	 * Applique un facteur d'�chelle sur ce builder. Toute les enveloppes
	 * construites par la suite seront soumises au changement d'�chelle. Le
	 * changement d'�chelle ne modifie pas la position et l'orientation en bodyspace
	 * des enveloppes cr��es. Pour cela, voir {@link #setTransform(Matrix4f)}.
	 * 
	 * @param scaleFactor
	 */
	public abstract void scale(float scaleFactor);

}
