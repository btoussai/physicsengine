package cataclysm.broadphase;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import cataclysm.Parallelizable;
import cataclysm.RayTest;
import cataclysm.parallel.PhysicsWork;
import cataclysm.parallel.PhysicsWorkerPool;
import cataclysm.parallel.PhysicsWorkerThread;

/**
 * A parallel version of {@link BroadPhaseTree}. Note that all methods are not
 * actually thread-safe. They are just implemented using a thread pool to
 * compute the result more quickly.
 * 
 * @author Briac
 *
 * @param <T>
 */
public class ParallelBroadPhaseTree<T> {

	/**
	 * Defines a callback testing if a node should be updated inside the bvh
	 * 
	 * @author Briac
	 *
	 * @param <T>
	 */
	public interface shouldUpdate<T> {
		/**
		 * Tests if a leaf node should be updated inside the bvh.
		 * 
		 * @param handle The handle of the node to be tested.
		 * @return true if the node should be replaced inside the bvh, in which case the
		 *         node's AABB must be updated
		 */
		public boolean test(T handle);
	}

	private final BroadPhaseTree<T> tree = new BroadPhaseTree<>();
	private final List<BroadPhaseTree<T>> subTrees = new ArrayList<>();

	public ParallelBroadPhaseTree() {

	}

	public void add(BroadPhaseNode<T> node) {
		tree.add(node);
	}

	public void remove(BroadPhaseNode<T> node) {
		tree.remove(node);
	}

	public void updateLeafs(PhysicsWorkerPool workers, shouldUpdate<T> shouldUpdate) {
		while (subTrees.size() < workers.getThreadCount()) {
			subTrees.add(new BroadPhaseTree<>());
		}

		List<PhysicsWork> w = new ArrayList<>(workers.getThreadCount());
		List<BroadPhaseNode<T>> nodes = new ArrayList<>();
		List<BroadPhaseNode<T>> leaves = new ArrayList<>();

		// we get all leaves and nodes at depth 5, which means we will get at most 32
		// sub-trees to
		// update
		getAllNodesAtDepth(tree.root, leaves, nodes, 5);

		for (int i = 0; i < workers.getThreadCount(); i++) {
			w.add(new PhysicsWork() {
				@Override
				public void run(PhysicsWorkerThread worker) {
					BroadPhaseTree<T> subTree = subTrees.get(worker.getThreadIndex());
					List<BroadPhaseNode<T>> subTreeLeaves = new ArrayList<>();

					// we perform a rotation to share the work more equally among the threads
					for (int k = worker.getThreadIndex(); k < nodes.size(); k += worker.getThreadCount()) {
						BroadPhaseNode<T> root = nodes.get(k);
						BroadPhaseNode<T> parent = root.parent;
						// we break the child/parent link
						root.parent = null;
						subTree.root = root;
						subTree.tempNode = null;
						updateSubTree(subTree, subTreeLeaves, shouldUpdate);
						// subtree.root may not be root anymore !
						// we create the child/parent link again
						subTree.root.parent = parent;
						if (parent.child1 == root) {
							parent.child1 = subTree.root;
						} else {
							parent.child2 = subTree.root;
						}
					}
					subTree.root = null;
					subTree.tempNode = null;

					worker.waitForTermination();
				}
			});
		}
		workers.scheduleWork(w, "updateBVH sub-trees", 1);
		workers.waitForTaskTermination();

		for (BroadPhaseNode<T> leaf : leaves) {
			if (shouldUpdate.test(leaf.getHandle())) {
				tree.update(leaf);
			}
		}
		for (BroadPhaseNode<T> node : nodes) {
			tree.update(node);
		}

	}

	/**
	 * Updates all leaves in a tree for which the predicate shouldUpdate is true.
	 * 
	 * @param t
	 * @param shouldUpdate
	 */
	private void updateSubTree(BroadPhaseTree<T> tree, List<BroadPhaseNode<T>> leaves, shouldUpdate<T> shouldUpdate) {

		leaves.clear();
		getAllLeavesNeedingUpdate(tree.root, leaves, shouldUpdate);

		for (BroadPhaseNode<T> node : leaves) {
			tree.update(node);
		}

	}

	private void getAllNodesAtDepth(BroadPhaseNode<T> node, List<BroadPhaseNode<T>> leaves,
			List<BroadPhaseNode<T>> nodes, int depth) {
		if (node.isLeaf) {
			leaves.add(node);
			return;
		}

		if (depth == 0) {
			nodes.add(node);
		} else {
			getAllNodesAtDepth(node.child1, leaves, nodes, depth - 1);
			getAllNodesAtDepth(node.child2, leaves, nodes, depth - 1);
		}
	}

	private void getAllLeavesNeedingUpdate(BroadPhaseNode<T> node, List<BroadPhaseNode<T>> leaves, shouldUpdate<T> shouldUpdate) {
		if (node.isLeaf) {
			if (shouldUpdate.test(node.getHandle())) {
				leaves.add(node);
			}
		} else {
			getAllLeavesNeedingUpdate(node.child1, leaves, shouldUpdate);
			getAllLeavesNeedingUpdate(node.child2, leaves, shouldUpdate);
		}
	}

	/**
	 * Deletes all nodes of the tree
	 */
	public void cleanUp() {
		tree.cleanUp();
	}

	/**
	 * @return The underlying bvh backing this parallel implementation
	 */
	public BroadPhaseTree<T> getTree() {
		return tree;
	}

	@Parallelizable
	public void rayTest(RayTest test) {
		tree.rayTest(test);
	}

	/**
	 * Retrieves all leaf nodes of the tree instersecting box.
	 * 
	 * @param box
	 * @param dest
	 */
	@Parallelizable
	public void boxTest(AABB box, Set<T> dest) {
		tree.boxTest(box, dest);
	}
}
