package cataclysm.broadphase;

import java.util.Set;

import cataclysm.Parallelizable;

/**
 * Représente un noeud du {@link BroadPhaseTree}.
 * 
 * @author Briac
 * @param <T>
 *
 */
public class BroadPhaseNode<T> {

	final AABB box;
	BroadPhaseNode<T> parent;
	BroadPhaseNode<T> child1;
	BroadPhaseNode<T> child2;
	final boolean isLeaf;
	float cost;

	private final T handle;

	/**
	 * Construit un nouveau noeud qui pourra être inséré dans l'arbre.
	 * 
	 * @param box
	 * @param handle
	 */
	public BroadPhaseNode(AABB box, T handle) {
		this.box = box;
		this.handle = handle;
		this.isLeaf = true;
	}

	/**
	 * Ce constructeur permet de construire les noeuds intérieur de l'arbre.
	 * 
	 * @param parent
	 * @param box
	 */
	BroadPhaseNode(BroadPhaseNode<T> parent, AABB box) {
		this.parent = parent;
		this.box = box;
		this.handle = null;
		this.isLeaf = false;
	}

	/**
	 * Récupère l'ensemble des feuilles de l'arbre en intersection avec l'AABB.
	 * 
	 * @param box
	 * @param dest
	 */
	@Parallelizable
	void boxTest(AABB box, Set<T> dest) {

		if (!AABB.intersect(this.box, box)) {
			return;
		}

		if (isLeaf) {
			dest.add(this.handle);
		} else {
			this.child1.boxTest(box, dest);
			this.child2.boxTest(box, dest);
		}
	}

	public AABB getBox() {
		return box;
	}

	public T getHandle() {
		return handle;
	}

	@Override
	public String toString() {
		return (isLeaf ? "Leaf " : "Node ") + super.toString();
	}

}
