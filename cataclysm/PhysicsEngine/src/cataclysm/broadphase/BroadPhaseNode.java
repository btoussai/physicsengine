package cataclysm.broadphase;

import java.util.HashSet;

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
	boolean isLeaf;
	float cost;
	
	private final T handle;

	public BroadPhaseNode(AABB box, T handle) {
		this.box = box;
		this.handle = handle;
	}

	public BroadPhaseNode(BroadPhaseNode<T> parent, AABB box) {
		this.parent = parent;
		this.box = box;
		this.handle = null;
	}

	/**
	 * R�cup�re l'ensemble des feuilles de l'arbre en intersection avec l'AABB.
	 * 
	 * @param box
	 * @param dest
	 */
	void boxTest(AABB box, HashSet<BroadPhaseNode<T>> dest) {

		if (!AABB.intersect(this.box, box)) {
			return;
		}

		if (isLeaf) {
			dest.add(this);
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
		// + id + " --> " + (parent != null ? parent.id : "null");
	}

}
