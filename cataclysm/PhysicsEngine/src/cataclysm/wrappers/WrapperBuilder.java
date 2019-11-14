package cataclysm.wrappers;

import org.lwjgl.util.vector.Matrix4f;

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
	 * n'influence que le tenseur d'inertie du rigidbody possédent l'enveloppe.
	 * 
	 * @param hollow
	 */
	public void setHollow(boolean hollow) {
		this.massProperties.setHollow(hollow);
	}

	/**
	 * Change la densité des enveloppes construites par ce builder. La masse des
	 * enveloppes sera déduite de la densité. <br>
	 * Si l'enveloppe est pleine, la densité correspond à la masse surfacique. <br>
	 * Si l'enveloppe est creuse, la densité correspond à la masse volumique.
	 * 
	 * @param density
	 */
	public void setDensity(float density) {
		this.massProperties.setDensity(density);
		this.massProperties.setUseDensity(true);
	}

	/**
	 * Change la masse des enveloppes construites par ce builder. La densité sera
	 * déduite de la masse. <br>
	 * Si l'enveloppe est pleine, la densité correspond à la masse surfacique. <br>
	 * Si l'enveloppe est creuse, la densité correspond à la masse volumique.
	 * 
	 * @param mass
	 */
	public void setMass(float mass) {
		this.massProperties.setMass(mass);
		this.massProperties.setUseDensity(false);
	}

	/**
	 * Construit une nouvelle enveloppe de collision pour le corps rigide passé en
	 * paramètre.
	 * 
	 * @param body Le corps rigide pour lequel l'enveloppe est construite.
	 * @param ID
	 * @return Une nouvelle enveloppe de collision.
	 */
	abstract Wrapper build(RigidBody body, long ID);

	/**
	 * Applique un facteur d'échelle sur ce builder. Toute les enveloppes
	 * construites par la suite seront soumises au changement d'échelle. Le
	 * changement d'échelle ne modifie pas la position et l'orientation en bodyspace
	 * des enveloppes créées. Pour cela, voir {@link #setTransform(Matrix4f)}.
	 * 
	 * @param scaleFactor
	 */
	protected abstract void scale(float scaleFactor);

}
