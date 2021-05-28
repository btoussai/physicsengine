package cataclysm.broadphase;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.IntFunction;

import cataclysm.Parallelizable;
import cataclysm.RayTest;
import cataclysm.datastructures.IntPriorityQueue;
import cataclysm.datastructures.IntStack;

/**
 * Defines a binary tree whose nodes are {@link BroadPhaseNode}. The tree is a
 * BVH (Bounding Volume Hierarchy), where each node is an AABB containing its
 * children nodes. <br>
 * 
 * @author Briac
 * @param <T> The leaf nodes have a reference to an object of type T
 *
 */
public class ArrayBasedBroadPhaseTree<T> {

	private static final boolean DEBUG = false;
	

	/**
	 * The root node of the tree. It contains every other node of the tree.
	 */
	int root = -1;

	private final IntPriorityQueue queue = new IntPriorityQueue(100,
			(left, right) -> Float.compare(getCost(left), getCost(right)));

	private static enum NODE_LAYOUT {
		minX, maxX, minY, maxY, minZ, maxZ, surfaceArea, parent, child1, child2, isLeaf, cost, handle, NODE_SIZE
	}

	/**
	 * Number of nodes which can be contained at most.
	 */
	private int capacity = 0;
	/**
	 * Number of nodes which are currently contained in the tree.
	 */
	private int size = 0;
	private int[] nodes;
	private T[] handles;
	private IntStack freeSpots = new IntStack(100);
	private IntFunction<T[]> arrayGenerator;
	
	public ArrayBasedBroadPhaseTree(IntFunction<T[]> arrayGenerator){
		nodes = new int[capacity];
		handles = arrayGenerator.apply(capacity);
		this.arrayGenerator = arrayGenerator;
	}

	private float getMinX(int index) {
		return Float.intBitsToFloat(nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.minX.ordinal()]);
	}

	private float getMinY(int index) {
		return Float.intBitsToFloat(nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.minY.ordinal()]);
	}

	private float getMinZ(int index) {
		return Float.intBitsToFloat(nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.minZ.ordinal()]);
	}

	private float getMaxX(int index) {
		return Float.intBitsToFloat(nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.maxX.ordinal()]);
	}

	private float getMaxY(int index) {
		return Float.intBitsToFloat(nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.maxY.ordinal()]);
	}

	private float getMaxZ(int index) {
		return Float.intBitsToFloat(nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.maxZ.ordinal()]);
	}

	private float getSurfaceArea(int index) {
		return Float.intBitsToFloat(nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.surfaceArea.ordinal()]);
	}

	private int getParent(int index) {
		return nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.parent.ordinal()];
	}

	private int getChild1(int index) {
		return nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.child1.ordinal()];
	}

	private int getChild2(int index) {
		return nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.child2.ordinal()];
	}

	private boolean isLeaf(int index) {
		return nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.isLeaf.ordinal()] != 0;
	}

	private float getCost(int index) {
		return Float.intBitsToFloat(nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.cost.ordinal()]);
	}

	private T getHandle(int index) {
		return handles[index];
	}

	private void setMinX(int index, float value) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.minX.ordinal()] = Float.floatToRawIntBits(value);
	}

	private void setMinY(int index, float value) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.minY.ordinal()] = Float.floatToRawIntBits(value);
	}

	private void setMinZ(int index, float value) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.minZ.ordinal()] = Float.floatToRawIntBits(value);
	}

	private void setMaxX(int index, float value) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.maxX.ordinal()] = Float.floatToRawIntBits(value);
	}

	private void setMaxY(int index, float value) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.maxY.ordinal()] = Float.floatToRawIntBits(value);
	}

	private void setMaxZ(int index, float value) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.maxZ.ordinal()] = Float.floatToRawIntBits(value);
	}

	private void setSurfaceArea(int index, float value) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.surfaceArea.ordinal()] = Float
				.floatToRawIntBits(value);
	}

	private void setParent(int index, int node) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.parent.ordinal()] = node;
	}

	private void setChild1(int index, int node) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.child1.ordinal()] = node;
	}

	private void setChild2(int index, int node) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.child2.ordinal()] = node;
	}

	private void setIsLeaf(int index, boolean b) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.isLeaf.ordinal()] = b ? 1 : 0;
	}

	private void setCost(int index, float value) {
		nodes[index * NODE_LAYOUT.NODE_SIZE.ordinal() + NODE_LAYOUT.cost.ordinal()] = Float.floatToRawIntBits(value);
	}

	private void setHandle(int index, T handle) {
		handles[index] = handle;
	}

	private void resize() {
		capacity = capacity * 2 + 1;
		int[] nodes = new int[capacity * NODE_LAYOUT.NODE_SIZE.ordinal()];
		T[] handles = arrayGenerator.apply(capacity);
		System.arraycopy(this.nodes, 0, nodes, 0, this.nodes.length);
		System.arraycopy(this.handles, 0, handles, 0, this.handles.length);
		this.nodes = nodes;
		this.handles = handles;
	}

	private float surfaceOfUnionAABB(int nodeA, int nodeB) {
		float minX = AABB.min(getMinX(nodeA), getMinX(nodeB));
		float minY = AABB.min(getMinY(nodeA), getMinY(nodeB));
		float minZ = AABB.min(getMinZ(nodeA), getMinZ(nodeB));
		float maxX = AABB.max(getMaxX(nodeA), getMaxX(nodeB));
		float maxY = AABB.max(getMaxY(nodeA), getMaxY(nodeB));
		float maxZ = AABB.max(getMaxZ(nodeA), getMaxZ(nodeB));
		float dx = maxX - minX;
		float dy = maxY - minY;
		float dz = maxZ - minZ;
		float surfaceArea = 2.0f * (dx * dy + dy * dz + dz * dx);
		return surfaceArea;
	}

	private void unionAABB(int nodeA, int nodeB, int dest) {
		float minX = AABB.min(getMinX(nodeA), getMinX(nodeB));
		float minY = AABB.min(getMinY(nodeA), getMinY(nodeB));
		float minZ = AABB.min(getMinZ(nodeA), getMinZ(nodeB));
		float maxX = AABB.max(getMaxX(nodeA), getMaxX(nodeB));
		float maxY = AABB.max(getMaxY(nodeA), getMaxY(nodeB));
		float maxZ = AABB.max(getMaxZ(nodeA), getMaxZ(nodeB));
		float dx = maxX - minX;
		float dy = maxY - minY;
		float dz = maxZ - minZ;
		float surfaceArea = 2.0f * (dx * dy + dy * dz + dz * dx);
		
		setMinX(dest, minX);
		setMinY(dest, minY);
		setMinZ(dest, minZ);
		setMaxX(dest, maxX);
		setMaxY(dest, maxY);
		setMaxZ(dest, maxZ);
		setSurfaceArea(dest, surfaceArea);
	}
	
	/**
	 * 
	 * @param nodeA
	 * @param nodeB
	 * @return true if the node A contains the node B.
	 */
	private boolean contains(int nodeA, int nodeB) {
		if (getMinX(nodeA) <= getMinX(nodeB) && getMaxX(nodeA) >= getMaxX(nodeB)) {
			if (getMinZ(nodeA) <= getMinZ(nodeB) && getMaxZ(nodeA) >= getMaxZ(nodeB)) {
				if (getMinY(nodeA) <= getMinY(nodeB) && getMaxY(nodeA) >= getMaxY(nodeB)) {
					return true;
				}
			}
		}
		return false;
	}
	
	/**
	 * 
	 * @param node
	 * @param box
	 * @return true if the node's box intersects the box.
	 */
	private boolean intersect(int node, AABB box) {
		if (getMaxX(node) > box.minX && getMinX(node) < box.maxX) {
			if (getMaxZ(node) > box.minZ && getMinZ(node) < box.maxZ) {
				if (getMaxY(node) > box.minY && getMinY(node) < box.maxY) {
					return true;
				}
			}
		}
		return false;
	}
	
	private int reserveSpot() {
		final int node;
		if(freeSpots.isEmpty()) {
			node = size++;
			if(size > capacity) {
				resize();
			}
		}else {
			node = freeSpots.pop();
		}
		return node;
	}
	
	private int initLeafNode(float minX, float minY, float minZ, float maxX, float maxY, float maxZ, T handle) {
		final int node = reserveSpot();
		
		setMinX(node, minX);
		setMinY(node, minY);
		setMinZ(node, minZ);
		setMaxX(node, maxX);
		setMaxY(node, maxY);
		setMaxZ(node, maxZ);
		setParent(node, -1);
		setChild1(node, -1);
		setChild2(node, -1);
		
		float dx = maxX - minX;
		float dy = maxY - minY;
		float dz = maxZ - minZ;
		float surfaceArea = 2.0f * (dx * dy + dy * dz + dz * dx);
		setSurfaceArea(node, surfaceArea);
		setIsLeaf(node, true);
		setHandle(node, handle);
		
		return node;
	}
	
	private int initInteriorNode(int parent, int child1, int child2) {
		final int node = reserveSpot();
		
		unionAABB(child1, child2, node);
		
		setParent(node, parent);
		setChild1(node, child1);
		setChild2(node, child2);
		setIsLeaf(node, false);
		setHandle(node, null);
		
		return node;
	}

	/**
	 * Adds a node in the tree
	 * @param minX 
	 * @param minY 
	 * @param minZ 
	 * @param maxX 
	 * @param maxY 
	 * @param maxZ 
	 * @param handle 
	 * @return 
	 */
	public int add(float minX, float minY, float minZ, float maxX, float maxY, float maxZ, T handle) {

		int node = initLeafNode(minX, minY, minZ, maxX, maxY, maxZ, handle);
		if (root == -1) {
			root = node;
		}else {
			add(node, root);
		}
		
		//System.out.println(this.toString());

		return node;
	}

	/**
	 * Adds a node in the tree.
	 * 
	 * @param node     The node to be inserted
	 * @param ancestor A node containing the new node or the root node
	 */
	private void add(int node, int ancestor) {

		// Stage 1: find the best sibling for the new leaf
		final int bestSibling = pickBestSibling(node, ancestor);

		// Stage 2: create a new parent
		final int oldParent = getParent(bestSibling);
		final int newParent = initInteriorNode(oldParent, node, bestSibling);
		
		if (oldParent != -1) {
			// The sibling was not the root
			if (getChild1(oldParent) == bestSibling) {
				setChild1(oldParent, newParent);
			} else {
				setChild2(oldParent, newParent);
			}
		} else {
			// The sibling was the root
			root = newParent;
		}
		setParent(bestSibling, newParent);
		setParent(node, newParent);

		// Stage 3: walk back up the tree refitting AABBs
		refitParents(node);

		if (DEBUG)
			if (!checkValidity()) {
				throw new IllegalStateException();
			}

	}
	
	private void markFree(int node) {
		freeSpots.push(node);
		size--;
		setParent(node, -1);
		setChild1(node, -1);
		setChild2(node, -1);
		setHandle(node, null);
	}
	
	/**
	 * Removes a leaf of the tree
	 * 
	 * @param node The node to be removed
	 */
	public void remove(int node) {
		setHandle(node, null);
		removeUpdate(node);
		markFree(node);
		if (node == root) {
			root = -1;
		}
	}

	/**
	 * Removes a leaf of the tree
	 * 
	 * @param node The node to be removed
	 */
	private void removeUpdate(int node) {
		final int parent = getParent(node);
		if (parent == -1) {
			if (node == root) {
				//Don't do anything here
			}else {
				throw new IllegalStateException("Error, trying to delete a node with no parent which is not the root");
			}
			return;
		}

		final int grandParent = getParent(parent);
		final int sibling;
		if (getChild1(parent) == node) {
			sibling = getChild2(parent);
		} else {
			sibling = getChild1(parent);
		}

		if (grandParent == -1) {
			setParent(sibling, -1);
			root = sibling;

			markFree(parent);
			return;
		}

		if (getChild1(grandParent) == parent) {
			setChild1(grandParent, sibling);
		} else {
			setChild2(grandParent, sibling);
		}
		setParent(sibling, grandParent);
		refitParents(sibling);

		if (DEBUG)
			if (!checkValidity()) {
				throw new IllegalStateException();
			}

		markFree(parent);
		return;
	}

	/**
	 * Updates a node.
	 * 
	 * @param node
	 * @param box 
	 */
	public void update(int node, AABB box) {
		setMinX(node, box.minX);
		setMinY(node, box.minY);
		setMinZ(node, box.minZ);
		setMaxX(node, box.maxX);
		setMaxY(node, box.maxY);
		setMaxZ(node, box.maxZ);
		
		int ancestor = getParent(node);
		
		if(!isLeaf(node)) {
			throw new IllegalStateException("Error, trying to update a non-leaf node");
		}

		if (ancestor == -1) {
			//That's the root node.
			return;
		}

		if (contains(ancestor, node)) {
			// no need to update anything
			return;
		}

		// check if the node box is at least contained within the root box
		if (!contains(root, node)) {
			removeUpdate(node);
			add(node, root);
			return;
		}

		// walk up the hierarchy until the box is contained in a parent box.
		do {
			ancestor = getParent(ancestor);
		} while (!contains(ancestor, node));

		removeUpdate(node);
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
		if (root != -1) {
			boxTest(root, box, dest);
		}
	}
	
	@Parallelizable
	private void boxTest(int node, AABB box, Set<T> dest) {

		if (!intersect(node, box)) {
			return;
		}

		if (isLeaf(node)) {
			dest.add(getHandle(node));
		} else {
			boxTest(getChild1(node), box, dest);
			boxTest(getChild2(node), box, dest);
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
	protected int pickBestSibling(int nodeToInsert, int ancestor) {

		if (isLeaf(ancestor)) {
			return ancestor;
		}

		// we first compute a cost metric (i.e. the surface area) of inserting the node
		// as a sibling of ancestor
		final float ancestorCost = surfaceOfUnionAABB(ancestor, nodeToInsert);
		setCost(ancestor, ancestorCost);

		float bestCost = ancestorCost;// the minimum cost so far
		int bestSibling = ancestor;// the sibling ensuring the minimum cost so far

		// inserting a node will increase the total surface area of the tree, here is a
		// lower bound on that cost
		float childLowerBoundCost = getSurfaceArea(nodeToInsert) + ancestorCost - getSurfaceArea(ancestor);
		if (childLowerBoundCost < bestCost) {
			final int child1 = getChild1(ancestor);
			setCost(child1, childLowerBoundCost);
			queue.add(child1);
			final int child2 = getChild2(ancestor);
			setCost(child2, childLowerBoundCost);
			queue.add(child2);
		}

		// we perform a breadth-first search in the children of ancestor
		while (!queue.isEmpty()) {

			final int current = queue.poll();
			final int parent = getParent(current);

			float unionCost = surfaceOfUnionAABB(current, nodeToInsert);

			float inheritedCost = getCost(parent) - getSurfaceArea(parent);
			float currentCost = unionCost + inheritedCost;
			setCost(current, currentCost);

			if (currentCost < bestCost) {
				queue.add(current);
				bestCost = currentCost;
				bestSibling = current;
			}

			if (isLeaf(current)) {
				continue;
			}

			childLowerBoundCost = getSurfaceArea(nodeToInsert) + currentCost - getSurfaceArea(current);
			if (childLowerBoundCost < bestCost) {
				final int child1 = getChild1(current);
				setCost(child1, childLowerBoundCost);
				queue.add(child1);
				final int child2 = getChild2(current);
				setCost(child2, childLowerBoundCost);
				queue.add(child2);
			}

		}

		return bestSibling;
	}

	/**
	 * 
	 * @param node
	 */
	private void refitParents(int node) {
		int current = getParent(node);
		while (current != -1) {
			unionAABB(getChild1(current), getChild2(current), current);
			rotateTree(current);
			current = getParent(current);
		}
	}

	/**
	 * Swaps a child node with a grand-child node if it diminishes the total surface
	 * area of the tree.
	 * 
	 * @param current
	 */
	private void rotateTree(int current) {
		// current isn't a leaf itself, so we know that both its children are not null.
		int child1 = getChild1(current);
		int child2 = getChild2(current);
		boolean isChild1Leaf = isLeaf(child1);
		boolean isChild2Leaf = isLeaf(child2);

		if (isChild1Leaf && isChild2Leaf) {
			// if both children are leaf nodes then current doesn't have any grand-children.
			return;
		}

		float bestDeltaSA = 0;
		float deltaSA = 0;

		int grandchild = -1;
		int child = -1;

		if (!isChild1Leaf) {
			deltaSA = surfaceOfUnionAABB(getChild2(child1), child2) - getSurfaceArea(child1);
			if (deltaSA < bestDeltaSA) {
				bestDeltaSA = deltaSA;
				grandchild = getChild1(child1);
				child = child2;
			}

			deltaSA = surfaceOfUnionAABB(getChild1(child1), child2) - getSurfaceArea(child1);
			if (deltaSA < bestDeltaSA) {
				bestDeltaSA = deltaSA;
				grandchild = getChild2(child1);
				child = child2;
			}
		}
		if (!isChild2Leaf) {
			deltaSA = surfaceOfUnionAABB(getChild2(child2), child1) - getSurfaceArea(child2);
			if (deltaSA < bestDeltaSA) {
				bestDeltaSA = deltaSA;
				grandchild = getChild1(child2);
				child = child1;
			}

			deltaSA = surfaceOfUnionAABB(getChild1(child2), child1) - getSurfaceArea(child2);
			if (deltaSA < bestDeltaSA) {
				bestDeltaSA = deltaSA;
				grandchild = getChild2(child2);
				child = child1;
			}
		}

		if (grandchild != -1) {
			int grandChildParent = getParent(grandchild);
			setParent(grandchild, current);
			setParent(child, grandChildParent);

			if (getChild1(grandChildParent) == grandchild) {
				setChild1(grandChildParent, child);
			} else {
				setChild2(grandChildParent, child);
			}
			unionAABB(getChild1(grandChildParent), getChild2(grandChildParent), grandChildParent);

			if (getChild1(current) == child) {
				setChild1(current, grandchild);
			} else {
				setChild2(current, grandchild);
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

		if (root != -1) {
			Queue<Integer> queue = new ArrayDeque<Integer>();
			queue.add(root);

			while (!queue.isEmpty()) {

				final int node = queue.poll();
				final int child1 = getChild1(node);
				final int child2 = getChild2(node);
				
				if (isLeaf(node)) {
					if (child1 != -1 || child2 != -1) {
						return false;
					}
					continue;
				} else {
					if (child1 == -1 || child2 == -1) {
						return false;
					}
				}

				if (!contains(node, child1))
					return false;
				queue.add(child1);
				if (!contains(node, child2))
					return false;
				queue.add(child2);

			}
		}

		return true;
	}

	@Override
	public String toString() {
		StringBuffer sb = new StringBuffer("BroadPhaseTree (size=" + size + ", capacity=" + capacity + ")[\n");

		if (root != -1) {
			Queue<Integer> list = new ArrayDeque<Integer>();
			list.add(root);
			Queue<Integer> nextList = new ArrayDeque<Integer>();

			while (!list.isEmpty()) {
				while (!list.isEmpty()) {
					final int node = list.poll();
					final int child1 = getChild1(node);
					final int child2 = getChild2(node);
					if (child1 != -1)
						nextList.add(child1);
					if (child2 != -1)
						nextList.add(child2);
					sb.append(node + "\t");
				}
				sb.append("\n");

				Queue<Integer> tempList = list;
				list = nextList;
				nextList = tempList;
			}

		}

		sb.append("]\n");

		return sb.toString();
	}

	private int depthExploration(int currentNode, List<Integer> list, int currentDepth,
			int maxDepth, int[] depths) {

		depths[currentDepth]++;

		if (currentDepth == maxDepth) {
			list.add(currentNode);
		}
		int leaves = 0;
		if (!isLeaf(currentNode)) {
			leaves += depthExploration(getChild1(currentNode), list, currentDepth + 1, maxDepth, depths);
			leaves += depthExploration(getChild2(currentNode), list, currentDepth + 1, maxDepth, depths);
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
	public int exploreBVH(List<Integer> list, int searchDepth) {
		int depths[] = new int[50];

		if (root != -1) {
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
		root = -1;
		capacity = 0;
		nodes = new int[0];
		handles = arrayGenerator.apply(0);
	}

	/**
	 * Aggregates the leaves of the tree in a list.
	 * 
	 * @param node A value of -1 designates the root node.
	 * @param leaves
	 */
	public void getLeaves(int node, List<T> leaves) {
		if (node == -1) {
			node = root;
		}
		if (isLeaf(node)) {
			leaves.add(getHandle(node));
		} else {
			getLeaves(getChild1(node), leaves);
			getLeaves(getChild2(node), leaves);
		}
	}
	
	public void readBox(int node, AABB box) {
		box.minX = getMinX(node);
		box.minY = getMinY(node);
		box.minZ = getMinZ(node);
		box.maxX = getMaxX(node);
		box.maxY = getMaxY(node);
		box.maxZ = getMaxZ(node);
	}

}
