package cataclysm.contact_creation;

import cataclysm.record.ReadWriteObject;
import cataclysm.record.RecordFile;

/**
 * This class holds some basic properties of the object's surface such as its
 * elasticity coefficient (how much energy is conserved during a collision) and its friction
 * coefficient.
 * 
 * @author Briac Toussaint
 *
 */
public final class ContactProperties implements ReadWriteObject {

	private float elasticity;
	private float friction;

	public ContactProperties(float elasticity, float friction) {
		this.elasticity = elasticity;
		this.friction = friction;
	}

	public ContactProperties(ContactProperties other) {
		this.elasticity = other.elasticity;
		this.friction = other.friction;
	}

	public float getElasticity() {
		return elasticity;
	}

	public ContactProperties setElasticity(float elasticity) {
		this.elasticity = elasticity;
		return this;
	}

	public float getFriction() {
		return friction;
	}

	public ContactProperties setFriction(float friction) {
		this.friction = friction;
		return this;
	}

	public ContactProperties(RecordFile f) {
		read(f);
	}

	@Override
	public void read(RecordFile f) {
		elasticity = f.readFloat();
		friction = f.readFloat();
	}

	@Override
	public void write(RecordFile f) {
		f.writeFloat(elasticity);
		f.writeFloat(friction);
	}

	public void set(ContactProperties contactProperties) {
		this.elasticity = contactProperties.elasticity;
		this.friction = contactProperties.friction;
	}

	@Override
	public int size() {
		return 8;
	}

}
