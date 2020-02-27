package math;

import math.vector.Vector3f;
import math.vector.Vector4f;

/**
 * Repr�sente un plan en 3D.
 * 
 * @author Briac
 *
 */
public class Plane {

	/**
	 * L'�quation du plan sous forme cart�sienne (ax+by+cz+d = 0) dans un vecteur
	 * (a, b, c, d)
	 */
	public Vector4f equation;

	/**
	 * Un point appartenant au plan.
	 */
	public Vector3f origin;

	/**
	 * Un vecteur unitaire perpendiculaire au plan.
	 */
	public Vector3f normal;

	/**
	 * Construit un plan � partir d'un point et d'un vecteur normal.
	 * 
	 * @param origin
	 * @param normal
	 */
	public Plane(Vector3f origin, Vector3f normal) {

		this.normal = new Vector3f(normal);
		this.origin = new Vector3f(origin);
		this.equation = new Vector4f(this.normal.x, this.normal.y, this.normal.z,
				-Vector3f.dot(this.normal, this.origin));

	}

	/**
	 * Construit un plan � partir de 3 points. La normale du plan pointera vers
	 * l'observateur si on passe de P1 � P2 � P3 en tournant dans le sens trigo du
	 * point de vue de l'observateur.
	 * 
	 * @param P1
	 * @param P2
	 * @param P3
	 */
	public Plane(Vector3f P1, Vector3f P2, Vector3f P3) {

		this.origin = new Vector3f(P1);
		this.normal = Vector3f.cross(Vector3f.sub(P2, P1), Vector3f.sub(P3, P1));
		this.normal.normalise();
		this.equation = new Vector4f(this.normal.x, this.normal.y, this.normal.z,
				-Vector3f.dot(this.normal, this.origin));

	}

	/**
	 * Teste si le plan fait face � l'observateur.
	 * 
	 * @param direction La direction dans laquelle regarde l'observateur.
	 * @return true si l'observateur voit la normale du plan point�e vers lui.
	 */
	public boolean isFrontFacingTo(Vector3f direction) {
		return (Vector3f.dot(direction, this.normal) <= 0);
	}

	/**
	 * Calcule la distance sign�e d'un point � ce plan. La distance est positive si
	 * le point est du m�me c�t� que la normale.
	 * 
	 * @param vertex Le point consid�r�.
	 * @return la distance sign�e du point au plan.
	 */
	public float signedDistanceTo(Vector3f vertex) {
		return Vector3f.dot(this.normal, vertex) + this.equation.w;
	}

	/**
	 * Calcule le projet� orthogonal d'un point sur ce plan.
	 * 
	 * @param vertex Le point consid�r�.
	 * @return le projet� orthogonal d'un point sur ce plan.
	 */
	public Vector3f project(Vector3f vertex) {
		float distance = signedDistanceTo(vertex);
		return new Vector3f(vertex.x - normal.x * distance, vertex.y - normal.y * distance,
				vertex.z - normal.z * distance);
	}

	public Vector4f getEquation() {
		return equation;
	}

	public Vector3f getOrigin() {
		return origin;
	}

	public Vector3f getNormal() {
		return normal;
	}

}
