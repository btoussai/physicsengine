package cataclysm.contact_creation;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import cataclysm.Epsilons;
import cataclysm.contact_creation.ContactFeature.FeatureType;
import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.ConvexHullWrapper.FloatLayout;
import math.vector.Vector3f;

/**
 * 
 * Cette classe contient une impl�mentation de l'algorithme SAT, d'apr�s la
 * pr�sentation "Dirk Gregorius � The Separating Axis Test", disponible ici:
 * <li><a href=
 * "https://box2d.org/downloads/">https://box2d.org/downloads/</a></li>
 *
 * @author Briac
 *
 */
class SAT {

	private final ContactFeature onA = new ContactFeature();
	private final ContactFeature onB = new ContactFeature();
	private final ReduceManifold reduceManifold = new ReduceManifold();
	private final PolygonClipping polygonClipping = new PolygonClipping();

	private static final boolean DEBUG = false;

	/**
	 * Teste l'intersection de deux enveloppes convexes.
	 * 
	 * @param hullA
	 * @param hullB
	 */
	void overlapTest(ConvexHullWrapper hullA, ConvexHullWrapper hullB, ContactZone contact) {

		if (DEBUG) {
			System.out.println("##	SAT:");
		}

		if (failFastCheck(hullA, hullB, contact)) {
			if (DEBUG) {
				System.out.println("Fail Fast check: early exit");
			}

			return;
		}

		faceCheck(hullA, hullB);
		float faceCheckDistanceA = penetrationDepth;
		int refFaceInA = referenceFace;

		if (DEBUG) {
			System.out.println("faceCheckDistanceA: " + faceCheckDistanceA);
		}

		if (faceCheckDistanceA >= 0.0f) {
			onA.setFromHullFace(referenceFace);
			onB.clean();
			hullA.getNormal(refFaceInA, faceNormal);
			contact.rebuild(faceNormal, faceCheckDistanceA, 0, onA, onB);
			contact.setNoCollision();
			return;
		}

		faceCheck(hullB, hullA);
		float faceCheckDistanceB = penetrationDepth;
		int refFaceInB = referenceFace;

		if (DEBUG) {
			System.out.println("faceCheckDistanceB: " + faceCheckDistanceB);
		}

		if (faceCheckDistanceB >= 0.0f) {
			onA.clean();
			onB.setFromHullFace(referenceFace);
			hullB.getNormal(refFaceInB, faceNormal);
			normal.negate();
			contact.rebuild(normal, faceCheckDistanceB, 0, onA, onB);
			contact.setNoCollision();
			return;
		}

		float edgeCheckDistance = Float.NEGATIVE_INFINITY;
		// If the face check gives a good enough result, we skip the edge check
		// completely unless the feature was an edge in the previous frame. This can
		// produce some false positives which will be handled
		// when clipping the contact faces against each other.
		if (contact.getFeatureOnA().getType() == FeatureType.HullEdge
				|| Math.max(faceCheckDistanceA, faceCheckDistanceB) < -Epsilons.ALLOWED_PENETRATION) {
			edgeCheck(hullA, hullB);
			edgeCheckDistance = penetrationDepth;
		}

		if (DEBUG) {
			System.out.println("edgeCheckDistance: " + edgeCheckDistance);
		}

		if (edgeCheckDistance > 0.0f) {
			onA.setFromHullEdge(contactEdgeA);
			onB.setFromHullEdge(contactEdgeB);
			contact.rebuild(normal, edgeCheckDistance, 0, onA, onB);
			contact.setNoCollision();
			return;
		}

		// we give a preference to face contacts
		if (faceCheckDistanceA > edgeCheckDistance - Epsilons.ALLOWED_PENETRATION
				|| faceCheckDistanceB > edgeCheckDistance - Epsilons.ALLOWED_PENETRATION) {

			// we check if face A and face B give similar results
			if (Math.abs(faceCheckDistanceA - faceCheckDistanceB) < Epsilons.ALLOWED_PENETRATION) {
				// we reassign so that features will stay the same across frames
				if (contact.getFeatureOnA().getType() == FeatureType.HullFace) {
					// A was the reference face and we keep it
					penetrationDepth = faceCheckDistanceA;
					referenceFace = refFaceInA;
					createFaceContact(contact, hullA, hullB, true);
				} else {
					// B was the reference face and we keep it
					penetrationDepth = faceCheckDistanceB;
					referenceFace = refFaceInB;
					createFaceContact(contact, hullA, hullB, false);
				}
			} else if (faceCheckDistanceA > faceCheckDistanceB) {
				// A is definitely better than B
				penetrationDepth = faceCheckDistanceA;
				referenceFace = refFaceInA;
				createFaceContact(contact, hullA, hullB, true);
			} else {
				// B is definitely better than A
				penetrationDepth = faceCheckDistanceB;
				referenceFace = refFaceInB;
				createFaceContact(contact, hullA, hullB, false);
			}
		} else {// it must be an edge contact at this point
			penetrationDepth = edgeCheckDistance;
			createEdgeContact(contact, hullA, hullB);
		}

	}

	/**
	 * Tests a previous separating axis.
	 * 
	 * @param contact
	 * @return true if the previous axis is still valid
	 */
	private boolean failFastCheck(ConvexHullWrapper hullA, ConvexHullWrapper hullB,
			ContactZone contact) {
		if (DEBUG) {
			System.out.println("Fail fast check: previous distance:" + contact.getPenetrationDepth());
			System.out.println("ContactFeature sur A:" + contact.getFeatureOnA().getType());
			System.out.println("ContactFeature sur B:" + contact.getFeatureOnB().getType());
		}

		if (contact.getPenetrationDepth() >= 0) {
			ContactFeature onA = contact.getFeatureOnA();
			ContactFeature onB = contact.getFeatureOnB();

			if (onA.getType() == FeatureType.HullFace) {
				int face = onA.getHullFeatureIndex();
				hullA.getNormal(face, normal);
				hullB.getSupport(normal, true, supportPoint);
				float distance = hullA.signedDistance(supportPoint, face);

				if (DEBUG) {
					System.out.println("pr�c�dente collision sur une face");
					System.out.println("referenceFace dans hullA : dist = " + distance);
				}

				if (distance > 0) {
					contact.setNoCollision();
					return true;
				}
			} else if (onB.getType() == FeatureType.HullFace) {
				int face = onB.getHullFeatureIndex();
				hullB.getNormal(face, normal);
				hullA.getSupport(normal, true, supportPoint);
				float distance = hullB.signedDistance(supportPoint, face);

				if (DEBUG) {
					System.out.println("pr�c�dente collision sur une face");
					System.out.println("referenceFace dans hullB : dist = " + distance);
				}

				if (distance > 0) {
					contact.setNoCollision();
					return true;
				}
			} else if (onA.getType() == FeatureType.HullEdge && onB.getType() == FeatureType.HullEdge) {
				if (DEBUG) {
					System.out.println("pr�c�dente collision sur une ar�te");
				}
				
				int edgeA = onA.getHullFeatureIndex();
				int edgeB = onB.getHullFeatureIndex();
				hullA.getEdgeVec(edgeA, vecEdgeA);				
				hullB.getEdgeVec(edgeB, vecEdgeB);

				if (!isMinkowskiFace(hullA, edgeA, vecEdgeA, hullB, edgeB, vecEdgeB)) {
					if (DEBUG) {
						System.out.println("Les arêtes ne forment plus une face sur la différence de Minkowski.");
					}
					return false;
				}

				centroidA.set(hullA.getCentroid());
				float distance = edgeDistance(hullA, edgeA, hullB, edgeB);

				if (DEBUG) {
					System.out.println("edgeDistance = " + distance);
				}

				if (distance > 0) {
					contact.setNoCollision();
					return true;
				}

			}

		}
		return false;
	}

	/**
	 * Builds the polygon of the contact zone
	 * 
	 * The reference face's normal gives the minimum penetration depth.
	 * 
	 * @param contact
	 * @param hullA
	 * @param hullB
	 * @param refFaceInHullA
	 */
	private void createFaceContact(ContactZone contact, ConvexHullWrapper hullA,
			ConvexHullWrapper hullB, boolean refFaceInHullA) {

		ConvexHullWrapper incident, reference;

		if (refFaceInHullA) {
			reference = hullA;
			incident = hullB;
		} else {
			reference = hullB;
			incident = hullA;
		}

		reference.getNormal(referenceFace, normal);
		int incidentFace = incident.getMostAntiParallelFace(normal);

		List<Vector3f> inputList = new ArrayList<Vector3f>();
		polygonClipping.clipIncidentFaceAgainstReferenceFace(incident, incidentFace, reference, referenceFace,
				inputList);

		if (inputList.isEmpty()) {
			// the two faces aren't overlapping which means it was a false positive,
			// it must be an edge contact
			edgeCheck(hullA, hullB);
			float edgeCheckDistance = penetrationDepth;

			if (DEBUG) {
				System.out.println(
						"false positive face contact, performed edgecheck \nedgeCheckDistance: " + edgeCheckDistance);
			}

			if (edgeCheckDistance > 0.0f) {
				onA.setFromHullEdge(contactEdgeA);
				onB.setFromHullEdge(contactEdgeB);
				contact.rebuild(normal, edgeCheckDistance, 0, onA, onB);
				contact.setNoCollision();
				return;
			}

			penetrationDepth = edgeCheckDistance;
			createEdgeContact(contact, hullA, hullB);
			return;
		}

		// we delete the points above the face's plane
		for (Iterator<Vector3f> it = inputList.iterator(); it.hasNext();) {
			Vector3f vertex = it.next();
			float distance = reference.signedDistance(vertex, referenceFace);
			if (distance > 5 * Epsilons.ALLOWED_PENETRATION) {
				it.remove();
			}
		}

		int contactCount = reduceManifold.reduceManifold(inputList, normal, contact);

		if (DEBUG) {
			System.out.println("createFaceContact (" + contactCount + " points)");
		}

		// On projette les points sur le plan de la face de référence
		for (int i = 0; i < contactCount; i++) {
			contact.getContactPoint(i, temp);
			float distance = reference.signedDistance(temp, referenceFace);
			temp.translate(normal, -distance);
			contact.setContactPointAndPenetrationDepth(i, temp.x, temp.y, temp.z, distance);
		}

		if (!refFaceInHullA) {
			normal.negate();
			onA.clean();
			onB.setFromHullFace(referenceFace);
		} else {
			onA.setFromHullFace(referenceFace);
			onB.clean();
		}

		contact.rebuild(normal, penetrationDepth, contactCount, onA, onB);
	}

	//
	// Some variables for createEdgeContact
	//
	private final Vector3f A = new Vector3f();
	private final Vector3f B = new Vector3f();
	private final Vector3f C = new Vector3f();
	private final Vector3f D = new Vector3f();

	private final Vector3f AB = new Vector3f();
	private final Vector3f CD = new Vector3f();
	private final Vector3f AC = new Vector3f();

	/**
	 * Builds a contact point as the middle point between the two edges.
	 * 
	 * @param contact
	 * @param hullA
	 * @param hullB
	 */
	private void createEdgeContact(ContactZone contact, ConvexHullWrapper hullA,
			ConvexHullWrapper hullB) {

		if (DEBUG) {
			System.out.println("createFaceContact");
		}

		hullA.get(FloatLayout.Vertices, hullA.getEdgeHead(contactEdgeA), A);
		hullA.get(FloatLayout.Vertices, hullA.getEdgeTail(contactEdgeA), B);

		hullB.get(FloatLayout.Vertices, hullB.getEdgeHead(contactEdgeB), C);
		hullB.get(FloatLayout.Vertices, hullB.getEdgeTail(contactEdgeB), D);

		Vector3f.sub(B, A, AB);
		Vector3f.sub(D, C, CD);
		Vector3f.sub(C, A, AC);

		float ABCD = Vector3f.dot(AB, CD);
		float ABAC = Vector3f.dot(AB, AC);
		float CDAC = Vector3f.dot(CD, AC);

		float AB2 = AB.lengthSquared();
		float CD2 = CD.lengthSquared();

		float det = AB2 * CD2 - ABCD * ABCD; // can't be zero since parallel edges are skipped in edgeCheck()

		float one_over_det = 1.0f / det;

		float t = one_over_det * (CD2 * ABAC - ABCD * CDAC);
		float s = one_over_det * (ABCD * ABAC - AB2 * CDAC);

		if (t < 0.0f) {
			t = 0.0f;
		} else if (t > 1.0f) {
			t = 1.0f;
		}

		if (s < 0.0f) {
			s = 0.0f;
		} else if (s > 1.0f) {
			s = 1.0f;
		}

		float x = 0.5f * (A.x + t * AB.x + C.x + s * CD.x);
		float y = 0.5f * (A.y + t * AB.y + C.y + s * CD.y);
		float z = 0.5f * (A.z + t * AB.z + C.z + s * CD.z);
		contact.setContactPointAndPenetrationDepth(0, x, y, z, penetrationDepth);
		onA.setFromHullEdge(contactEdgeA);
		onB.setFromHullEdge(contactEdgeB);
		contact.rebuild(normal, penetrationDepth, 1, onA, onB);

	}

	// some variables for face check.
	private float penetrationDepth;
	private int referenceFace;
	private final Vector3f supportPoint = new Vector3f();
	private final Vector3f faceNormal = new Vector3f();

	/**
	 * Finds a separating axis between the two bodies. That axis is a face nomral of
	 * hull1.
	 * 
	 * @param hull1
	 * @param hull2
	 */
	private void faceCheck(ConvexHullWrapper hull1, ConvexHullWrapper hull2) {
		penetrationDepth = Float.NEGATIVE_INFINITY;

		for (int face = 0; face < hull1.faceCount; face++) {

			hull1.getNormal(face, faceNormal);
			hull2.getSupport(faceNormal, true, supportPoint);

			float distance = hull1.signedDistance(supportPoint, face);

			if (distance > penetrationDepth) {
				penetrationDepth = distance;
				referenceFace = face;
				if (distance >= 0) {
					return;
				}
			}

		}
	}

	//
	// Some variables for edgeCheck
	//
	private final Vector3f centroidA = new Vector3f();
	private int contactEdgeA;
	private final Vector3f vecEdgeA = new Vector3f();
	private int contactEdgeB;
	private final Vector3f vecEdgeB = new Vector3f();
	private final Vector3f edgeAxEdgeB = new Vector3f();
	private final Vector3f temp = new Vector3f();
	private final Vector3f normal = new Vector3f();

	/**
	 * Finds a separating axis between the two bodies. That axis is a cross product
	 * of two edges in A and B.
	 * 
	 * @param hullA
	 * @param hullB
	 */
	private void edgeCheck(ConvexHullWrapper hullA, ConvexHullWrapper hullB) {
		penetrationDepth = Float.NEGATIVE_INFINITY;
		centroidA.set(hullA.getCentroid());

		for (int edgeA = 0; edgeA < hullA.edgeCount; edgeA += 2) {

			hullA.getEdgeVec(edgeA, vecEdgeA);

			for (int edgeB = 0; edgeB < hullB.edgeCount; edgeB += 2) {
				hullB.getEdgeVec(edgeB, vecEdgeB);

				if (!isMinkowskiFace(hullA, edgeA, vecEdgeA, hullB, edgeB, vecEdgeB)) {
					continue;
				}

				float distance = edgeDistance(hullA, edgeA, hullB, edgeB);

				if (distance > penetrationDepth) {
					penetrationDepth = distance;
					normal.set(edgeAxEdgeB);
					contactEdgeA = edgeA;
					contactEdgeB = edgeB;
					if (distance >= 0) {
						return;
					}
				}

			}
		}

	}

	/**
	 * Computes the distance between the two edges
	 * 
	 * The following variables must be set: centroidA, vecEdgeA, vecEdgeB
	 * 
	 * @param edgeA
	 * @param edgeB
	 * @return
	 */
	private float edgeDistance(ConvexHullWrapper hullA, int edgeA, ConvexHullWrapper hullB,
			int edgeB) {
		Vector3f.cross(vecEdgeA, vecEdgeB, edgeAxEdgeB);
		float length2 = edgeAxEdgeB.lengthSquared();

		if (length2 < (1.0f - Epsilons.PARALLEL_LIMIT_2) * vecEdgeA.lengthSquared() * vecEdgeB.lengthSquared()) {// skip
																													// parallel
																													// edges.
			return Float.NEGATIVE_INFINITY;
		}
		float one_over_length = 1.0f / (float) Math.sqrt(length2);

		hullA.get(FloatLayout.Vertices, hullA.getEdgeTail(edgeA), temp);
		Vector3f.sub(temp, centroidA, temp);
		if (Vector3f.dot(edgeAxEdgeB, temp) < 0) {
			one_over_length = -one_over_length;
		}

		edgeAxEdgeB.x *= one_over_length;
		edgeAxEdgeB.y *= one_over_length;
		edgeAxEdgeB.z *= one_over_length;

		hullA.get(FloatLayout.Vertices, hullA.getEdgeTail(edgeA), temp);
		hullB.sub(FloatLayout.Vertices, hullB.getEdgeTail(edgeB), temp, temp);
		float distance = Vector3f.dot(edgeAxEdgeB, temp);

		return distance;
	}

	/**
	 * Checks if the cross product of the two edges can be an axis of separation.
	 * 
	 * @param edgeA
	 * @param vecEdgeA
	 * @param edgeB
	 * @param vecEdgeB
	 * @return true if the axis is valid
	 */
	private boolean isMinkowskiFace(ConvexHullWrapper hullA, int edgeA, Vector3f vecEdgeA,
			ConvexHullWrapper hullB, int edgeB, Vector3f vecEdgeB) {

		float CBA = hullB.dot(FloatLayout.FaceNormals, hullB.getEdgeFace(edgeB), vecEdgeA);
		float DBA = hullB.dot(FloatLayout.FaceNormals, hullB.getEdgeAdjacentFace(edgeB), vecEdgeA);
		float ADC = -hullA.dot(FloatLayout.FaceNormals, hullA.getEdgeFace(edgeA), vecEdgeB);
		float BDC = -hullA.dot(FloatLayout.FaceNormals, hullA.getEdgeAdjacentFace(edgeA), vecEdgeB);

		// float CBA = Vector3f.dot(edgeB.getFaceNormal(), vecEdgeA);
		// float DBA = Vector3f.dot(edgeB.getAdjacentFaceNormal(), vecEdgeA);
		// float ADC = -Vector3f.dot(edgeA.getFaceNormal(), vecEdgeB);
		// float BDC = -Vector3f.dot(edgeA.getAdjacentFaceNormal(), vecEdgeB);

		return CBA * DBA < 0 && ADC * BDC < 0 && CBA * BDC > 0;
	}

}
