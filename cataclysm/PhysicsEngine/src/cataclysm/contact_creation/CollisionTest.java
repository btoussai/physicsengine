package cataclysm.contact_creation;

import java.util.List;

import cataclysm.CataclysmCallbacks;
import cataclysm.wrappers.CapsuleWrapper;
import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.SphereWrapper;
import cataclysm.wrappers.Wrapper;

/**
 * Cette classe permet de tester la collision de deux solides et construit une
 * zone de contact le cas �ch�ant.
 * 
 * @author Briac
 *
 */
public class CollisionTest {

	private final CollideWrapperTriangle wrapperCollider = new CollideWrapperTriangle();
	private final CollideSpheres collideSpheres = new CollideSpheres();
	private final CollideSphereCapsule collideSphereCapsule = new CollideSphereCapsule();
	private final CollideSphereHull collideSphereHull = new CollideSphereHull();
	private final CollideCapsules collideCapsules = new CollideCapsules();
	private final CollideCapsuleHull collideCapsuleHull = new CollideCapsuleHull();
	private final CollideHulls collideHulls = new CollideHulls();

	/**
	 * Instancie un nouveau testeur de collision. Il permet de construire une zone
	 * de contact entre deux solides ou entre un solide et un maillage statique.
	 */
	public CollisionTest() {

	}

	/**
	 * Teste la collision entre une enveloppe convexe et un ensemble de triangles.
	 * 
	 * @param wrapper
	 * @param callbacks
	 * @param meshContacts
	 */
	public void meshContacts(Wrapper wrapper, CataclysmCallbacks callbacks,
			List<AbstractSingleBodyContact> meshContacts) {
		wrapperCollider.test(wrapper, callbacks, meshContacts);
	}

	/**
	 * Teste la collision d'une paire d'objets et construit des zones de contact le
	 * cas échéant.
	 * 
	 * @param contact
	 * @param callbacks
	 * @param bodyContacts
	 */
	public void bodyContacts(AbstractDoubleBodyContact contact, CataclysmCallbacks callbacks,
			List<AbstractDoubleBodyContact> bodyContacts) {

		ContactArea area = contact.area;
		convexContact(contact.getWrapperA(), contact.getWrapperB(), area);
		if (area.isCollisionOccuring()) {
			bodyContacts.add(contact);
			if (callbacks.getOnCollision() != null)
				callbacks.getOnCollision().accept(contact.getWrapperA(), contact.getWrapperB());
		} else {
			contact.resetImpulses();
		}

	}

	/**
	 * Teste si deux solides convexes sont en contact et construit une zone de
	 * contact le cas échéant.
	 * 
	 * @param A Le premier solide.
	 * @param B Le second solide.
	 */
	private void convexContact(Wrapper A, Wrapper B, ContactArea contact) {

		// System.out.println("Wrapper vs Wrapper");

		switch (A.getType()) {
		case Sphere:
			if (B.getType() == Wrapper.Type.Sphere) {
				collideSpheres.test((SphereWrapper) A, (SphereWrapper) B, contact);
				break;
			} else if (B.getType() == Wrapper.Type.Capsule) {
				collideSphereCapsule.test((SphereWrapper) A, (CapsuleWrapper) B, contact);
				break;
			} else {
				collideSphereHull.test((SphereWrapper) A, (ConvexHullWrapper) B, contact);
				break;
			}
		case Capsule:
			if (B.getType() == Wrapper.Type.Capsule) {
				collideCapsules.test((CapsuleWrapper) A, (CapsuleWrapper) B, contact);
				break;
			} else {
				collideCapsuleHull.test((CapsuleWrapper) A, (ConvexHullWrapper) B, contact);
				break;
			}
		case ConvexHull:
			collideHulls.test((ConvexHullWrapper) A, (ConvexHullWrapper) B, contact);
			break;
		}

	}

}
