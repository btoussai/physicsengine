package cataclysm.broadphase;

import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Queue;

/**
 * Représente un arbre dont les noeuds sont des {@link BroadPhaseNode}. L'arbre
 * représente une BVH (Bounding Volume Hierarchy). Il s'agit d'un arbre binaire
 * dans lequel chaque noeud est une AABB englobant tous ses noeuds fils.
 * 
 * @author Briac
 * @param <T>
 *
 */
public class BroadPhaseTree<T> {

	private static final boolean DEBUG = false;

	private BroadPhaseNode<T> root;
	private BroadPhaseNode<T> tempNode;

	private final AABB tempBox = new AABB();
	private final PriorityQueue<BroadPhaseNode<T>> queue = new PriorityQueue<BroadPhaseNode<T>>(
			(left, right) -> Float.compare(left.cost, right.cost));

	/**
	 * Ajoute un noeud dans l'arbre.
	 * 
	 * @param node Le noeud à insérer.
	 */
	public void add(BroadPhaseNode<T> node) {

		if (root == null) {
			root = node;
			return;
		}

		add(node, root);
	}

	private void add(BroadPhaseNode<T> node, BroadPhaseNode<T> ancestor) {

		// Stage 1: find the best sibling for the new leaf
		BroadPhaseNode<T> bestSibling = pickBestSibling(node, ancestor);

		// Stage 2: create a new parent
		BroadPhaseNode<T> oldParent = bestSibling.parent;
		BroadPhaseNode<T> newParent = null;
		if (tempNode == null) {
			newParent = new BroadPhaseNode<T>(oldParent, AABB.union(node.box, bestSibling.box));
		} else {
			newParent = tempNode;
			tempNode = null;
			newParent.parent = oldParent;
			AABB.union(node.box, bestSibling.box, newParent.box);
		}
		if (oldParent != null) {
			// The sibling was not the root
			if (oldParent.child1 == bestSibling) {
				oldParent.child1 = newParent;
			} else {
				oldParent.child2 = newParent;
			}
		} else {
			// The sibling was the root
			root = newParent;
		}
		newParent.child1 = bestSibling;
		newParent.child2 = node;
		bestSibling.parent = newParent;
		node.parent = newParent;

		// Stage 3: walk back up the tree refitting AABBs
		refitParents(node);

		if (DEBUG)
			if (!checkValidity()) {
				throw new IllegalStateException();
			}

	}

	/**
	 * 
	 * Supprime une feuille de l'arbre.
	 * 
	 * @param node Le noeud à retirer.
	 */
	public void remove(BroadPhaseNode<T> node) {
		if (node.parent == null) {
			if (node == root) {
				root = null;
			}
			return;
		}

		BroadPhaseNode<T> parent = node.parent;
		BroadPhaseNode<T> grandParent = parent.parent;
		BroadPhaseNode<T> sibling = null;
		if (parent.child1 == node) {
			sibling = parent.child2;
		} else {
			sibling = parent.child1;
		}

		if (grandParent == null) {
			root = sibling;
			root.parent = null;

			parent.parent = parent.child1 = parent.child2 = null;
			tempNode = parent;
			return;
		}

		if (grandParent.child1 == parent) {
			grandParent.child1 = sibling;
		} else {
			grandParent.child2 = sibling;
		}
		sibling.parent = grandParent;

		refitParents(sibling);

		node.parent = null;

		if (DEBUG)
			if (!checkValidity()) {
				throw new IllegalStateException();
			}

		parent.parent = parent.child1 = parent.child2 = null;
		tempNode = parent;
		return;
	}

	/**
	 * Met à jour un noeud. Si le noeud est nouveau, il est inséré dans l'arbre.
	 * 
	 * @param node
	 */
	public void update(BroadPhaseNode<T> node) {
		if(node.parent == null) {
			remove(node);
			add(node);
			return;
		}

		AABB box = node.getBox();
		BroadPhaseNode<T> ancestor = node.parent;
		
		if(ancestor.box.contains(box)) {
			return;
		}
		
		ancestor = ancestor.parent;
		while (ancestor != null) {
			if (ancestor.box.contains(box)) {
				break;
			} else {
				ancestor = ancestor.parent;
			}
		}

		if (ancestor == null) {
			remove(node);
			add(node);
		} else {
			remove(node);
			add(node, ancestor);
		}

	}

	public void rayTest() {

	}

	/**
	 * Récupère l'ensemble des feuilles de l'arbre en intersection avec l'AABB.
	 * 
	 * @param box
	 * @param dest
	 */
	public void boxTest(AABB box, HashSet<T> dest) {
		if (root != null) {
			root.boxTest(box, dest);
		}
	}

	/**
	 * Sélectionne un noeud parmi les descendants de ancestor qui deviendra le "frère" de nodeToInsert.
	 * @param nodeToInsert
	 * @param ancestor
	 * @return
	 */
	private BroadPhaseNode<T> pickBestSibling(BroadPhaseNode<T> nodeToInsert, BroadPhaseNode<T> ancestor) {

		if (ancestor.isLeaf) {
			return ancestor;
		}

		AABB.union(ancestor.box, nodeToInsert.box, tempBox);
		ancestor.cost = tempBox.getSurfaceArea();

		float bestCost = ancestor.cost;
		BroadPhaseNode<T> bestSibling = ancestor;

		float childLowerBoundCost = nodeToInsert.box.getSurfaceArea() + ancestor.cost - ancestor.box.getSurfaceArea();
		if (childLowerBoundCost < bestCost) {
			ancestor.child1.cost = childLowerBoundCost;
			queue.add(ancestor.child1);
			ancestor.child2.cost = childLowerBoundCost;
			queue.add(ancestor.child2);
		}

		while (!queue.isEmpty()) {

			BroadPhaseNode<T> current = queue.poll();
			BroadPhaseNode<T> parent = current.parent;

			AABB.union(current.box, nodeToInsert.box, tempBox);
			float unionCost = tempBox.getSurfaceArea();

			float inheritedCost = parent.cost - parent.box.getSurfaceArea();
			current.cost = unionCost + inheritedCost;

			if (current.cost < bestCost) {
				queue.add(current);
				bestCost = current.cost;
				bestSibling = current;
			}

			if (current.isLeaf) {
				continue;
			}

			childLowerBoundCost = nodeToInsert.box.getSurfaceArea() + current.cost - current.box.getSurfaceArea();
			if (childLowerBoundCost < bestCost) {
				current.child1.cost = childLowerBoundCost;
				queue.add(current.child1);
				current.child2.cost = childLowerBoundCost;
				queue.add(current.child2);
			}

		}

		return bestSibling;
	}

	private void refitParents(BroadPhaseNode<T> node) {
		BroadPhaseNode<T> current = node.parent;
		while (current != null) {
			AABB.union(current.child1.box, current.child2.box, current.box);
			rotateTree(current);
			current = current.parent;
		}
	}

	/**
	 * Echange un noeud fils avec un noeud petit-fils si cela diminue l'aire totale
	 * de l'arbre.
	 * 
	 * @param current
	 */
	private void rotateTree(BroadPhaseNode<T> current) {
		// current isn't a leaf itself, so both its children are not null.
		if (current.child1.isLeaf && current.child2.isLeaf) {
			return;
		}

		float bestDeltaSA = 0;
		float deltaSA = 0;

		BroadPhaseNode<T> grandchild = null;
		BroadPhaseNode<T> child = null;

		if (!current.child1.isLeaf) {
			AABB.union(current.child1.child2.box, current.child2.box, tempBox);
			deltaSA = tempBox.getSurfaceArea() - current.child1.box.getSurfaceArea();
			if (deltaSA < bestDeltaSA) {
				bestDeltaSA = deltaSA;
				grandchild = current.child1.child1;
				child = current.child2;
			}

			AABB.union(current.child1.child1.box, current.child2.box, tempBox);
			deltaSA = tempBox.getSurfaceArea() - current.child1.box.getSurfaceArea();
			if (deltaSA < bestDeltaSA) {
				bestDeltaSA = deltaSA;
				grandchild = current.child1.child2;
				child = current.child2;
			}
		}
		if (!current.child2.isLeaf) {
			AABB.union(current.child2.child2.box, current.child1.box, tempBox);
			deltaSA = tempBox.getSurfaceArea() - current.child2.box.getSurfaceArea();
			if (deltaSA < bestDeltaSA) {
				bestDeltaSA = deltaSA;
				grandchild = current.child2.child1;
				child = current.child1;
			}

			AABB.union(current.child2.child1.box, current.child1.box, tempBox);
			deltaSA = tempBox.getSurfaceArea() - current.child2.box.getSurfaceArea();
			if (deltaSA < bestDeltaSA) {
				bestDeltaSA = deltaSA;
				grandchild = current.child2.child2;
				child = current.child1;
			}
		}

		if (grandchild != null) {
			BroadPhaseNode<T> grandChildParent = grandchild.parent;
			grandchild.parent = current;
			child.parent = grandChildParent;

			if (grandChildParent.child1 == grandchild) {
				grandChildParent.child1 = child;
			} else {
				grandChildParent.child2 = child;
			}
			AABB.union(grandChildParent.child1.box, grandChildParent.child2.box, grandChildParent.box);

			if (current.child1 == child) {
				current.child1 = grandchild;
			} else {
				current.child2 = grandchild;
			}
		}

	}

	private boolean checkValidity() {

		if (root != null) {
			Queue<BroadPhaseNode<T>> queue = new ArrayDeque<BroadPhaseNode<T>>();
			queue.add(root);

			while (!queue.isEmpty()) {

				BroadPhaseNode<T> node = queue.poll();
				if (node.child1 != null) {
					if (!node.box.contains(node.child1.box))
						return false;
					queue.add(node.child1);
				}
				if (node.child2 != null) {
					if (!node.box.contains(node.child2.box))
						return false;
					queue.add(node.child2);
				}
			}
		}

		return true;
	}

	@Override
	public String toString() {
		StringBuffer sb = new StringBuffer("BroadPhaseTree[\n");

		if (root != null) {
			Queue<BroadPhaseNode<T>> list = new ArrayDeque<BroadPhaseNode<T>>();
			list.add(root);
			Queue<BroadPhaseNode<T>> nextList = new ArrayDeque<BroadPhaseNode<T>>();

			while (!list.isEmpty()) {
				while (!list.isEmpty()) {
					BroadPhaseNode<T> node = list.poll();
					if (node.child1 != null)
						nextList.add(node.child1);
					if (node.child2 != null)
						nextList.add(node.child2);
					sb.append(node + "\t");
				}
				sb.append("\n");

				Queue<BroadPhaseNode<T>> tempList = list;
				list = nextList;
				nextList = tempList;
			}

		}

		sb.append("]\n");

		return sb.toString();
	}

	public void cleanUp() {
		root = null;
	}

}
