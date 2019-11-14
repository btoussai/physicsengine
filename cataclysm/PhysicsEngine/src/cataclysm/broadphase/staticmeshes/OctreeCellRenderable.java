package cataclysm.broadphase.staticmeshes;

import org.lwjgl.util.vector.Vector3f;

/**
 * Représente une boite affichable permettant de visualiser une cellule de l'octree.
 * @author Briac
 *
 */
public class OctreeCellRenderable {

	public Vector3f center;
	public float half_size;
	public int depth;
	
	public OctreeCellRenderable(Vector3f center, float half_size, int depth) {
		this.center = center;
		this.half_size = half_size;
		this.depth = depth;
	}
	
	
	
}
