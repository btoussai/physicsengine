package cataclysm.contact_creation;

import cataclysm.wrappers.SphereWrapper;
import math.vector.Vector3f;

/**
 * Permet de tester la collision entre deux spheres.
 * @author Briac
 *
 */
class CollideSpheres {

	private final Vector3f AB = new Vector3f();
	private final Vector3f normal = new Vector3f();

	private final ContactFeature onA = new ContactFeature();
	private final ContactFeature onB = new ContactFeature();

	void test(SphereWrapper A, SphereWrapper B, ContactArea contact) {

		Vector3f.sub(B.getCentroid(), A.getCentroid(), AB);

		float distance = AB.length();
		float depth = distance - A.getRadius() - B.getRadius();

		if (distance > 1E-3f && depth < 0) {

			normal.set(AB.x / distance, AB.y / distance, AB.z / distance);

			float toMiddlePoint = A.getRadius() + 0.5f * depth;
			
			Vector3f collisionPoint = contact.contactPoints[0];
			collisionPoint.set(A.getCentroid());
			collisionPoint.x += normal.x * toMiddlePoint;
			collisionPoint.y += normal.y * toMiddlePoint;
			collisionPoint.z += normal.z * toMiddlePoint;
			
			onA.setFrom(A.getCentroid());
			onB.setFrom(B.getCentroid());
			
			contact.penetrations[0] = depth;
			contact.rebuild(normal, depth, 1, onA, onB);
		}else {
			contact.setNoCollision();
		}

	}

}
