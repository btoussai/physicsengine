package cataclysm.contact_creation;

import cataclysm.Epsilons;
import cataclysm.wrappers.CapsuleWrapper;
import math.vector.Vector3f;

/**
 * Permet de tester la collision entre deux capsules.
 * @author Briac
 *
 */
class CollideCapsules {
	
	private static final boolean DEBUG = false;

	private final Vector3f normal = new Vector3f();
	
	private final ContactFeature onA = new ContactFeature();
	private final ContactFeature onB = new ContactFeature();

	/**
	 * Les deux extr�mit�s de la capsule2, apr�s clipping contre AB.
	 */
	private final Vector3f Cprime = new Vector3f();
	private final Vector3f Dprime = new Vector3f();

	private final Vector3f AB = new Vector3f();
	private final Vector3f CD = new Vector3f();
	private final Vector3f AC = new Vector3f();
	private final Vector3f AD = new Vector3f();

	/**
	 * Les points minimisant la distance entre les deux capsules.
	 */
	private final Vector3f C1 = new Vector3f();
	private final Vector3f C2 = new Vector3f();

	private final Vector3f C1C2 = new Vector3f();

	void test(CapsuleWrapper capsule1, CapsuleWrapper capsule2, ContactArea contact) {

		Vector3f A = capsule1.getCenter1();
		Vector3f B = capsule1.getCenter2();

		Vector3f C = capsule2.getCenter1();
		Vector3f D = capsule2.getCenter2();
		
		Vector3f.sub(B, A, AB);
		Vector3f.sub(D, C, CD);
		Vector3f.sub(C, A, AC);

		float ABCD = Vector3f.dot(AB, CD);
		float ABAC = Vector3f.dot(AB, AC);
		float CDAC = Vector3f.dot(CD, AC);

		float AB2 = AB.lengthSquared();
		float CD2 = CD.lengthSquared();

		float det = AB2 * CD2 - ABCD * ABCD;

		if (det < (1.0f-Epsilons.PARALLEL_LIMIT_2) * AB2 * CD2) {// the capsules are parallel.
			if(DEBUG)System.out.println("Les capsules sont parall�les.");

			Vector3f.sub(D, A, AD);
			float ABAD = Vector3f.dot(AB, AD);

			if (ABAC < 0.0f && ABAD < 0.0f) {// capsule2 is below capsule1
				if(DEBUG)System.out.println("Les capsules sont d�cal�es.");

				C1.set(A);
				if (ABAC > ABAD) {
					C2.set(C);
				} else {
					C2.set(D);
				}
				sphereTest(capsule1, capsule2, contact);

			} else if (ABAC > AB2 && ABAD > AB2) {// capsule1 is below capsule2
				if(DEBUG)System.out.println("Les capsules sont d�cal�es.");

				C1.set(B);
				if (ABAC > ABAD) {
					C2.set(D);
				} else {
					C2.set(C);
				}
				sphereTest(capsule1, capsule2, contact);

			} else {// the capsules are next to each other
				if(DEBUG)System.out.println("Les capsules sont empil�es.");

				// clip C against AB:
				float offset = 0;
				if (ABAC < 0.0f) {
					offset = -ABAC / ABCD;
					Cprime.set(C.x + offset * CD.x, C.y + offset * CD.y, C.z + offset * CD.z);
				} else if (ABAC > AB2) {
					offset = (AB2 - ABAC) / ABCD;
					Cprime.set(C.x + offset * CD.x, C.y + offset * CD.y, C.z + offset * CD.z);
				} else {
					Cprime.set(C);
				}

				// clip D against AB:
				if (ABAD < 0.0f) {
					offset = -ABAD / ABCD;
					Dprime.set(C.x + offset * CD.x, C.y + offset * CD.y, C.z + offset * CD.z);
				} else if (ABAD > AB2) {
					offset = (AB2 - ABAD) / ABCD;
					Dprime.set(C.x + offset * CD.x, C.y + offset * CD.y, C.z + offset * CD.z);
				} else {
					Dprime.set(D);
				}

				// the normal is computed from the new C.
				Vector3f.sub(Cprime, A, AC);
				ABAC = Vector3f.dot(AB, AC);
				float correction = ABAC / AB2;
				normal.set(AC.x - correction * AB.x, AC.y - correction * AB.y,
						AC.z - correction * AB.z);

				float distance2 = normal.lengthSquared();
				
				float R1 = capsule1.getRadius();
				float R2 = capsule2.getRadius();
				float R1R2 = R1 + R2;
				
				if (distance2 > Epsilons.MIN_LENGTH_2) {

					float distance = (float) Math.sqrt(distance2);
					normal.set(normal.x / distance, normal.y / distance, normal.z / distance);

					float depthC = distance - R1R2;
					
					Vector3f.sub(Dprime, A, AD);
					float depthD = Vector3f.dot(normal, AD) - R1R2;
					
					if(DEBUG) {
						System.out.println("StackingTest: normal = " + normal);
						System.out.println("depthC: " + depthC + " depthD: " + depthD);
					}
					
					boolean C_above = depthC > 0;
					boolean D_above = depthD > 0;
					
					if(C_above && D_above) {
						contact.setNoCollision();
						return;
					}

					float toMiddlePointC = R2 + 0.5f * depthC;
					contact.contactPoints[0].set(Cprime.x - toMiddlePointC * normal.x,
							Cprime.y - toMiddlePointC * normal.y, Cprime.z - toMiddlePointC * normal.z);
					
					float toMiddlePointD = R2 + 0.5f * depthD;
					contact.contactPoints[1].set(Dprime.x - toMiddlePointD * normal.x,
							Dprime.y - toMiddlePointD * normal.y, Dprime.z - toMiddlePointD * normal.z);

					
					onA.setFrom(capsule1.getCenter1(), capsule1.getCenter2());
					onB.setFrom(capsule2.getCenter1(), capsule2.getCenter2());
					
					contact.penetrations[0] = depthC;
					contact.penetrations[1] = depthD;
					contact.rebuild(normal, Math.min(depthC, depthD), 2, onA, onB);
				}else {
					contact.setNoCollision();
				}

			}

		} else {// the capsules are antiparallel enough.
			if(DEBUG)System.out.println("Les capsules ne sont pas parall�les.");
			
			float one_over_det = 1.0f / det;

			float t = one_over_det * (CD2 * ABAC - ABCD * CDAC);
			float s = one_over_det * (ABCD * ABAC - AB2 * CDAC);

			if (t < 0.0f) {
				t = 0.0f;
				s = -CDAC / CD2;
			} else if (t > 1.0f) {
				t = 1.0f;
				s = (-CDAC + ABCD) / CD2;
			}

			if (s < 0.0f) {
				t = ABAC / AB2;
				s = 0.0f;

				if (t < 0.0f) {
					t = 0.0f;
				} else if (t > 1.0f) {
					t = 1.0f;
				}
			} else if (s > 1.0f) {
				t = (ABAC + ABCD) / AB2;
				s = 1.0f;

				if (t < 0.0f) {
					t = 0.0f;
				} else if (t > 1.0f) {
					t = 1.0f;
				}
			}

			C1.set(A.x + t * AB.x, A.y + t * AB.y, A.z + t * AB.z);
			C2.set(C.x + s * CD.x, C.y + s * CD.y, C.z + s * CD.z);
			
			sphereTest(capsule1, capsule2, contact);
		}

	}

	private void sphereTest(CapsuleWrapper capsule1, CapsuleWrapper capsule2, ContactArea contact) {
		Vector3f.sub(C2, C1, C1C2);

		float distance2 = C1C2.lengthSquared();
		
		if(DEBUG) {
			System.out.println("SphereTest:    C1: " + C1 + ", C2: " + C2);
			System.out.println("distance: " + (float)Math.sqrt(distance2));
		}

		float R1 = capsule1.getRadius();
		float R2 = capsule2.getRadius();
		float R1R2 = R1 + R2;
		
		if (distance2 > Epsilons.MIN_LENGTH_2 && distance2 < R1R2*R1R2) {

			float distance = (float) Math.sqrt(distance2);
			float depth = distance - R1R2;

			normal.set(C1C2.x / distance, C1C2.y / distance, C1C2.z / distance);

			float toMiddlePoint = R1 + 0.5f * depth;

			contact.contactPoints[0].set(C1.x + normal.x * toMiddlePoint, C1.y + normal.y * toMiddlePoint,
					C1.z + normal.z * toMiddlePoint);
			
			onA.setFrom(capsule1.getCenter1(), capsule1.getCenter2());
			onB.setFrom(capsule2.getCenter1(), capsule2.getCenter2());
			
			contact.penetrations[0] = depth;
			contact.rebuild(normal, depth, 1, onA, onB);
		}else {
			contact.setNoCollision();
		}

	}

}
