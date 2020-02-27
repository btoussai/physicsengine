package cataclysm.quickHull;

import math.vector.Vector3f;

/**
 * Repr�sente un sommet d'une structure half edge
 * 
 * @author Briac
 *
 */
public class Vertex {

	/**
	 * La position en model-space du sommet.
	 */
	private Vector3f position;

	public Vertex(Vector3f position) {
		this.position = position;
	}

	public Vector3f getPosition() {
		return position;
	}

	public void setPosition(Vector3f position) {
		this.position = position;
	}

	public String toString() {
		return position.toString();
	}

}
