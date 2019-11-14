package cataclysm.contact_creation;

public class ContactProperties {
	
	private float elasticity;
	private float friction;
	private float stickiness;
	
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

	public float getStickiness() {
		return stickiness;
	}

	public ContactProperties setStickiness(float stickiness) {
		this.stickiness = stickiness;
		return this;
	}
	

}
