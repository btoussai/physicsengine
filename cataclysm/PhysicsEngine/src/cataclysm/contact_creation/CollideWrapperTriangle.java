package cataclysm.contact_creation;

import java.util.List;

import cataclysm.CataclysmCallbacks;
import cataclysm.wrappers.CapsuleWrapper;
import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.SphereWrapper;
import cataclysm.wrappers.TriangleAsHull;
import cataclysm.wrappers.Wrapper;

/**
 * Permet de tester la collision entre un wrapper et un ensemble de triangles.
 * 
 * @author Briac
 *
 */
class CollideWrapperTriangle {

	/**
	 * Permet de transformer les triangles en enveloppe convexe.
	 */
	private final TriangleAsHull triangleHull = TriangleAsHull.buildNew();

	private final CollideSphereHull collideSphereHull = new CollideSphereHull();
	private final CollideCapsuleHull collideCapsuleHull = new CollideCapsuleHull();
	private final CollideHulls collideHulls = new CollideHulls();

	/**
	 * Teste la collision entre une enveloppe convexe et un ensemble de triangles.
	 * 
	 * @param wrapper
	 * @param callbacks
	 * @param meshContacts
	 */
	public void test(Wrapper wrapper, CataclysmCallbacks callbacks, List<AbstractSingleBodyContact> meshContacts) {

		// System.out.println("Wrapper vs Triangle");

		updateContacts(wrapper);

		List<AbstractSingleBodyContact> contacts = wrapper.getMeshContacts();
		// contacts.sort(Comparator.comparingDouble((c) ->
		// c.area.getPenetrationDepth()));

		for (AbstractSingleBodyContact contact : contacts) {
			if (!contact.area.isCollisionOccuring()) {
				contact.resetImpulses();
				continue;
			}

			if (callbacks.getOnCollisionWithGround() != null) {
				callbacks.getOnCollisionWithGround().accept(wrapper);
			}

			meshContacts.add(contact);
		}

	}

	/**
	 * Met à jour les contacts avec les triangles de ce wrapper.
	 * 
	 * @param wrapper
	 */
	private void updateContacts(Wrapper wrapper) {
		int a = 0;

		for (AbstractSingleBodyContact contact : wrapper.getMeshContacts()) {
			triangleHull.setFrom(contact.getTriangle());

			switch (wrapper.getType()) {
			case Sphere:
				collideSphereHull.test((SphereWrapper) wrapper, triangleHull, contact.area);
				break;
			case Capsule:
				collideCapsuleHull.test((CapsuleWrapper) wrapper, triangleHull, contact.area);
				break;
			case ConvexHull:
				collideHulls.test((ConvexHullWrapper) wrapper, triangleHull, contact.area);
				break;
			default:
				throw new IllegalArgumentException("Unknown enum value: " + wrapper.getType());
			}
		}
	}

}
