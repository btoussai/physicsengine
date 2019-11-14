package cataclysm.wrappers;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Vector3f;

import cataclysm.DefaultParameters;
import cataclysm.PhysicsWorld;
import cataclysm.broadphase.AABB;
import cataclysm.broadphase.BroadPhaseNode;
import cataclysm.broadphase.BroadPhaseTree;
import cataclysm.broadphase.Pair;
import cataclysm.broadphase.PairManager;
import cataclysm.constraints.AbstractConstraint;
import cataclysm.constraints.AnchorPoint;
import cataclysm.datastructures.BufferedManager;

/**
 * Contient l'ensemble des objets de la simulation.
 * 
 * @author Briac
 *
 */
public class RigidBodyManager extends BufferedManager<RigidBody> {

	private static final boolean DEBUG = false;

	private final BroadPhaseTree<Wrapper> bvh;
	private final PairManager pairManager;
	private final PhysicsWorld world;

	private final float PADDING;
	private final float PADDING_SQUARED;

	public RigidBodyManager(PhysicsWorld world) {
		this.world = world;
		this.bvh = new BroadPhaseTree<Wrapper>();
		this.pairManager = new PairManager(world.getParameters().getCollisionFilter());

		PADDING = world.getParameters().getPadding();
		PADDING_SQUARED = PADDING * PADDING;
	}

	/**
	 * Ajoute un corps rigide dans la simulation.
	 * 
	 * @param transform La position et la rotation de l'objet en world-space.
	 * @param params    Les paramètres par défaut
	 * @param builders  Les enveloppes de l'objet.
	 * 
	 * @return L'objet nouvellement créé.
	 */
	public RigidBody newBody(Matrix4f transform, DefaultParameters params, WrapperBuilder... builders) {
		RigidBody body = new RigidBody(transform, params, this.generator, builders);
		addElement(body);
		return body;
	}

	@Override
	protected void processAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed) {

		HashSet<BroadPhaseNode<Wrapper>> set = new HashSet<BroadPhaseNode<Wrapper>>();

		for (RigidBody body : removed) {
			for (Wrapper wrapper : body.getWrappers()) {
				set.clear();
				bvh.boxTest(wrapper.getNode().getBox(), set);
				for (BroadPhaseNode<Wrapper> node : set) {
					long ID = Pair.buildID(node.getHandle().getID(), wrapper.getID());
					pairManager.removePair(ID);
				}
				bvh.remove(wrapper.getNode());
			}
		}

		for (RigidBody body : added) {
			for (Wrapper wrapper : body.getWrappers()) {

				BroadPhaseNode<Wrapper> newNode = wrapper.getNode();
				bvh.add(newNode);

				// System.out.println("After adding " + newNode + " pos: " +
				// wrapper.getCentroid());
				// System.out.println(bvh);

				set.clear();
				bvh.boxTest(newNode.getBox(), set);
				for (BroadPhaseNode<Wrapper> node : set) {
					pairManager.addPair(node.getHandle(), wrapper);
				}

				// System.out.println(pairManager);
			}
		}

	}

	@Override
	protected void cleanAddedAndRemovedElements(List<RigidBody> added, List<RigidBody> removed) {
		List<AbstractConstraint> contraintsToDelete = new ArrayList<AbstractConstraint>();
		for (RigidBody body : removed) {
			for (Wrapper w : body)
				generator.freeID(w.getID());
			for (AnchorPoint point : body.getAnchorPoints()) {
				contraintsToDelete.add(point.getConstraint());
			}
		}
		super.cleanAddedAndRemovedElements(added, removed);

		contraintsToDelete.forEach(constraint -> world.deleteConstraint(constraint));
	}

	@Override
	protected void internalUpdate() {
		HashSet<BroadPhaseNode<Wrapper>> oldPairs = new HashSet<BroadPhaseNode<Wrapper>>();
		HashSet<BroadPhaseNode<Wrapper>> newPairs = new HashSet<BroadPhaseNode<Wrapper>>();
		List<BroadPhaseNode<Wrapper>> temp = new ArrayList<BroadPhaseNode<Wrapper>>();

		for (RigidBody body : this) {
			if (body.isSleeping())
				continue;
			for (Wrapper wrapper : body.getWrappers()) {
				AABB box = wrapper.getNode().getBox();
				Vector3f centroid = wrapper.getCentroid();

				float sx = 0.5f * (box.min.x + box.max.x) - centroid.x;
				float sy = 0.5f * (box.min.y + box.max.y) - centroid.y;
				float sz = 0.5f * (box.min.z + box.max.z) - centroid.z;

				float d2 = sx * sx + sy * sy + sz * sz;

				if (d2 < PADDING_SQUARED) {
					continue;
				}

				// System.out.println("Updating " + box.getBroadPhaseNode() + " d2: " + d2 + "
				// pos: " + centroid);
				updatePairs(wrapper, oldPairs, newPairs, temp);

			}
		}

		if (DEBUG) {
			checkPairs();
		}

	}

	private void checkPairs() {

		for (RigidBody b : this) {
			Wrapper w = b.getWrappers().get(0);
			this.forEach((b2) -> {
				Wrapper w2 = b.getWrappers().get(0);
				if (w == w2)
					return;
				if (AABB.intersect(w.getNode().getBox(), w2.getNode().getBox())) {
					if (!pairManager.contains(Pair.buildID(w.getID(), w2.getID()))) {
						throw new IllegalStateException("Erreur dans les paires d'objets en collision !");
					}
				}
			});

		}

	}

	/**
	 * Recalcule les paires d'objets en collision avec box.
	 * 
	 * @param wrapper
	 * @param oldPairs
	 * @param newPairs
	 * @param temp
	 */
	private void updatePairs(Wrapper wrapper, HashSet<BroadPhaseNode<Wrapper>> oldPairs, HashSet<BroadPhaseNode<Wrapper>> newPairs,
			List<BroadPhaseNode<Wrapper>> temp) {

		 BroadPhaseNode<Wrapper> node = wrapper.getNode();
		 
		oldPairs.clear();
		bvh.boxTest(node.getBox(), oldPairs);
		bvh.remove(node);

		// System.out.println("Après suppression " + bvh);
		// System.out.println("oldPairs:");
		// oldPairs.forEach(e -> System.out.print(e + " \t"));
		// System.out.println("\n");

		wrapper.placeBox(PADDING);
		bvh.add(node);
		newPairs.clear();
		bvh.boxTest(node.getBox(), newPairs);

		// System.out.println("Après ajout " + bvh);
		// System.out.println("newPairs:");
		// newPairs.forEach(e -> System.out.print(e + " \t"));
		// System.out.println("\n");

		temp.clear();
		temp.addAll(oldPairs);
		oldPairs.removeAll(newPairs);
		newPairs.removeAll(temp);

		oldPairs.forEach(n -> {
			long ID = Pair.buildID(n.getHandle().getID(), wrapper.getID());
			pairManager.removePair(ID);
		});
		newPairs.forEach(n -> {
			pairManager.addPair(n.getHandle(), wrapper);
		});
	}

	public PairManager getPairs() {
		return pairManager;
	}

	/**
	 * Ajoute les points d'ancrage de la contrainte aux rigidbody reliés par la
	 * constrainte.
	 * 
	 * @param constraint
	 */
	public void addConstraint(AbstractConstraint constraint) {
		AnchorPoint pointA = constraint.getPointA();
		if (!pointA.isStatic()) {
			pointA.getBody().addAnchorPoint(pointA);
		}
		AnchorPoint pointB = constraint.getPointB();
		if (!pointB.isStatic()) {
			pointB.getBody().addAnchorPoint(pointB);
		}
	}

	/**
	 * Retire les points d'ancrage de la contrainte des rigidbody reliés par la
	 * constrainte.
	 * 
	 * @param constraint
	 */
	public void removeConstraint(AbstractConstraint constraint) {
		AnchorPoint pointA = constraint.getPointA();
		if (!pointA.isStatic()) {
			pointA.getBody().removeAnchorPoint(pointA);
		}
		AnchorPoint pointB = constraint.getPointA();
		if (!pointB.isStatic()) {
			pointB.getBody().removeAnchorPoint(pointB);
		}
	}

	@Override
	public void cleanUp() {
		super.cleanUp();
		pairManager.cleanUp();
		bvh.cleanUp();
	}

}
