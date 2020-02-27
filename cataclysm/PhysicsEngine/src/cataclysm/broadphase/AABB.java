package cataclysm.broadphase;

import math.VectorOps;
import math.vector.Vector3f;

/**
 * Repr�sente une bo�te align�e sur les axes xyz.
 * 
 * @author Briac
 *
 */
public class AABB {

	public final Vector3f min = new Vector3f();
	public final Vector3f max = new Vector3f();
	private float surfaceArea = 0;

	/**
	 * Initialise l'AABB avec min = max = vec3(0, 0, 0);
	 */
	public AABB() {

	}

	/**
	 * Construit une AABB � partir d'un point central et d'une taille. <br>
	 * L'AABB mesurera 2*radius selon chaque direction.
	 * 
	 * @param center
	 * @param radius
	 */
	public AABB(Vector3f center, float radius) {
		this.min.set(center.x - radius, center.y - radius, center.z - radius);
		this.max.set(center.x + radius, center.y + radius, center.z + radius);
	}

	/**
	 * Teste si cette AABB contient entièrement other.<br>
	 * Note: {@code this.contains(this)} vaut toujours true.
	 * 
	 * @param other
	 * @return true si other est à l'intérieur de cette AABB.
	 */
	public boolean contains(AABB other) {
		if (min.x <= other.min.x && max.x >= other.max.x) {
			if (min.z <= other.min.z && max.z >= other.max.z) {
				if (min.y <= other.min.y && max.y >= other.max.y) {
					return true;
				}
			}
		}
		return false;
	}

	/**
	 * Calcule l'union de deux AABB.
	 * 
	 * @param left
	 * @param right
	 * @return l'AABB la plus petite telle que left & right soient comprises �
	 *         l'int�rieur.
	 */
	public static AABB union(AABB left, AABB right) {
		AABB dest = new AABB();
		union(left, right, dest);
		return dest;
	}

	/**
	 * Calcule l'union de deux AABB et place le r�sultat dans dest.
	 * 
	 * @param left
	 * @param right
	 * @param dest
	 */
	public static void union(AABB left, AABB right, AABB dest) {
		VectorOps.min(left.min, right.min, dest.min);
		VectorOps.max(left.max, right.max, dest.max);
		dest.computeSurfaceArea();
	}

	public static boolean intersect(AABB left, AABB right) {
		if (left.max.x > right.min.x && left.min.x < right.max.x) {
			if (left.max.z > right.min.z && left.min.z < right.max.z) {
				if (left.max.y > right.min.y && left.min.y < right.max.y) {
					return true;
				}
			}
		}
		return false;
	}

	float getSurfaceArea() {
		return surfaceArea;
	}

	public float computeSurfaceArea() {
		float dx = max.x - min.x;
		float dy = max.y - min.y;
		float dz = max.z - min.z;
		surfaceArea = 2.0f * (dx * dy + dy * dz + dz * dx);
		return surfaceArea;
	}

	@Override
	public String toString() {
		return "AABB[ " + min + " | " + max + "]";
	}

}
