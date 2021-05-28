package cataclysm.broadphase;

import java.util.ArrayDeque;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;

import cataclysm.Parallelizable;
import cataclysm.RayTest;

/**
 * Defines a binary tree whose nodes are {@link BroadPhaseNode}. The tree is a
 * BVH (Bounding Volume Hierarchy), where each node is an AABB containing its
 * children nodes. <br>
 * 
 * @author Briac
 * @param <T> The leaf nodes have a reference to an object of type T
 *
 */
public class BroadPhaseTree<T> {

	private static final boolean DEBUG = false;

	/**
	 * The root node of the tree. It contains every other node of the tree.
	 */
	BroadPhaseNode<T> root;
	BroadPhaseNode<T> tempNode;

	private final AABB tempBox = new AABB();
	private final PriorityQueue<BroadPhaseNode<T>> queue = new PriorityQueue<BroadPhaseNode<T>>(
			(left, right) -> Float.compare(left.cost, right.cost));

	/**
	 * Adds a node in the tree
	 * 
	 * @param node The node to be inserted
	 */
	public void add(BroadPhaseNode<T> node) {

		if (root == null) {
			root = node;
			return;
		}

		add(node, root);
	}

	/**
	 * Adds a node in the tree.
	 * 
	 * @param node     The node to be inserted
	 * @param ancestor A node containing the new node or the root node
	 */
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
	 * Removes a leaf of the tree
	 * 
	 * @param node The node to be removed
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
	 * Updates a node. If the node is new, it will be inserted in the tree instead.
	 * 
	 * @param node
	 */
	public void update(BroadPhaseNode<T> node) {
		BroadPhaseNode<T> ancestor = node.parent;

		if (ancestor == null) {
			remove(node);
			add(node);
			return;
		}

		AABB box = node.getBox();

		if (ancestor.box.contains(box)) {
			// no need to update anything
			return;
		}

		// check if the box is at least contained within the root box
		if (!root.box.contains(box)) {
			remove(node);
			add(node);
			return;
		}

		// walk up the hierarchy until the box is contained in a parent box.
		do {
			ancestor = ancestor.parent;
		} while (!ancestor.box.contains(box));

		remove(node);
		add(node, ancestor);

	}

	public void rayTest(RayTest test) {
		throw new IllegalStateException("Not implemented");
	}

	/**
	 * Retrieves all leaf nodes of the tree instersecting box.
	 * 
	 * @param box
	 * @param dest
	 */
	@Parallelizable
	public void boxTest(AABB box, Set<T> dest) {
		if (root != null) {
			root.boxTest(box, dest);
		}
	}

	/**
	 * Selects a node among the children of ancestor which will become a sibling of
	 * nodeToInsert.
	 * 
	 * @param nodeToInsert
	 * @param ancestor     A node containing the box of nodeToInsert or the root
	 *                     node
	 * @return The sibling node for nodeToInsert
	 */
	protected BroadPhaseNode<T> pickBestSibling(BroadPhaseNode<T> nodeToInsert, BroadPhaseNode<T> ancestor) {

		if (ancestor.isLeaf) {
			return ancestor;
		}

		// we first compute a cost metric (i.e. the surface area) of inserting the node
		// as a sibling of ancestor
		AABB.union(ancestor.box, nodeToInsert.box, tempBox);
		ancestor.cost = tempBox.getSurfaceArea();

		float bestCost = ancestor.cost;// the minimum cost so far
		BroadPhaseNode<T> bestSibling = ancestor;// the sibling ensuring the minimum cost so far

		// inserting a node will increase the total surface area of the tree, here is a
		// lower bound on that cost
		float childLowerBoundCost = nodeToInsert.box.getSurfaceArea() + ancestor.cost - ancestor.box.getSurfaceArea();
		if (childLowerBoundCost < bestCost) {
			ancestor.child1.cost = childLowerBoundCost;
			queue.add(ancestor.child1);
			ancestor.child2.cost = childLowerBoundCost;
			queue.add(ancestor.child2);
		}

		// we perform a breadth-first search in the children of ancestor
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

	/**
	 * 
	 * @param node
	 */
	private void refitParents(BroadPhaseNode<T> node) {
		BroadPhaseNode<T> current = node.parent;
		while (current != null) {
			AABB.union(current.child1.box, current.child2.box, current.box);
			rotateTree(current);
			current = current.parent;
		}
	}

	/**
	 * Swaps a child node with a grand-child node if it diminishes the total surface
	 * area of the tree.
	 * 
	 * @param current
	 */
	private void rotateTree(BroadPhaseNode<T> current) {
		// current isn't a leaf itself, so we know that both its children are not null.

		if (current.child1.isLeaf && current.child2.isLeaf) {
			// if both children are leaf nodes then current doesn't have any grand-children.
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

	/**
	 * Checks the validity of the tree: <br>
	 * i.e. that every box contains its children
	 * 
	 * @return true if the tree is valid
	 */
	private boolean checkValidity() {

		if (root != null) {
			Queue<BroadPhaseNode<T>> queue = new ArrayDeque<BroadPhaseNode<T>>();
			queue.add(root);

			while (!queue.isEmpty()) {

				BroadPhaseNode<T> node = queue.poll();
				if (node.isLeaf) {
					if (node.child1 != null || node.child2 != null) {
						return false;
					}
				} else {
					if (node.child1 == null || node.child2 == null) {
						return false;
					}
				}

				if (!node.box.contains(node.child1.box))
					return false;
				queue.add(node.child1);
				if (!node.box.contains(node.child2.box))
					return false;
				queue.add(node.child2);

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

	private int depthExploration(BroadPhaseNode<T> currentNode, List<BroadPhaseNode<T>> list, int currentDepth,
			int maxDepth, int[] depths) {

		depths[currentDepth]++;

		if (currentDepth == maxDepth) {
			list.add(currentNode);
		}
		int leaves = 0;
		if (!currentNode.isLeaf) {
			leaves += depthExploration(currentNode.child1, list, currentDepth + 1, maxDepth, depths);
			leaves += depthExploration(currentNode.child2, list, currentDepth + 1, maxDepth, depths);
		} else {
			leaves = 1;
		}

		return leaves;
	}

	/**
	 * Performs an exploration of the tree.
	 * 
	 * @param list        Fills the list with all nodes having a depth equal to
	 *                    searchDepth. The root node has a depth of 0.
	 * @param searchDepth
	 * @return the maximum depth of the bvh
	 */
	public int exploreBVH(List<BroadPhaseNode<T>> list, int searchDepth) {
		int depths[] = new int[50];

		if (root != null) {
			int leaves = depthExploration(root, list, 0, searchDepth, depths);

			for (int i = 0; i < depths.length; i++) {
				if (depths[i] == 0) {
					// System.out.println("Total leaves = " + leaves);
					return i - 1;
				}
				// System.out.println("Depth " + i + " : " + depths[i]);
			}
		}
		return 0;
	}

	public void cleanUp() {
		root = null;
	}

	/**
	 * Aggregates the leaves of the tree in a list.
	 * 
	 * @param node
	 * @param leaves
	 */
	public void getLeaves(BroadPhaseNode<T> node, List<T> leaves) {
		if (node == null) {
			node = root;
		}
		if (node.isLeaf) {
			leaves.add(node.getHandle());
		} else {
			getLeaves(node.child1, leaves);
			getLeaves(node.child2, leaves);
		}
	}

}
