package cataclysm.contact_creation;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.wrappers.CapsuleWrapper;
import cataclysm.wrappers.SphereWrapper;

/**
 * Permet de tester la collision entre une sphere et une capsule.
 * @author Briac
 *
 */
class CollideSphereCapsule {

	private static final Vector3f axis = new Vector3f();
	private static final Vector3f toSphere = new Vector3f();
	private static final Vector3f normal = new Vector3f();
	private static final Vector3f closestToSphere = new Vector3f();
	private static final Vector3f AB = new Vector3f();
	
	private static final ContactFeature onA = new ContactFeature();
	private static final ContactFeature onB = new ContactFeature();

	static void test(SphereWrapper sphere, CapsuleWrapper capsule, ContactArea contact) {

		Vector3f.sub(capsule.getCenter2(), capsule.getCenter1(), axis);
		Vector3f.sub(sphere.getCentroid(), capsule.getCenter1(), toSphere);

		float t = Vector3f.dot(axis, toSphere) / axis.lengthSquared();
		if (t < 0.0f) {
			t = 0.0f;
		} else if (t > 1.0f) {
			t = 1.0f;
		}

		closestToSphere.set(capsule.getCenter1());
		closestToSphere.x += t * axis.x;
		closestToSphere.y += t * axis.y;
		closestToSphere.z += t * axis.z;

		Vector3f.sub(closestToSphere, sphere.getCentroid(), AB);

		float distance = AB.length();
		float depth = distance - sphere.getRadius() - capsule.getRadius();

		if (distance > 1E-3f && depth < 0) {

			normal.set(AB.x / distance, AB.y / distance, AB.z / distance);

			float toMiddlePoint = sphere.getRadius() + 0.5f * depth;

			Vector3f collisionPoint = contact.contactPoints[0];
			collisionPoint.set(sphere.getCentroid());
			collisionPoint.x += normal.x * toMiddlePoint;
			collisionPoint.y += normal.y * toMiddlePoint;
			collisionPoint.z += normal.z * toMiddlePoint;
			
			onA.setFrom(sphere.getCentroid());
			onB.setFrom(capsule.getCenter1(), capsule.getCenter2());
			
			contact.penetrations[0] = depth;
			contact.rebuild(normal, depth, 1, onA, onB);
		}else {
			contact.setNoCollision();
		}

	}

}
