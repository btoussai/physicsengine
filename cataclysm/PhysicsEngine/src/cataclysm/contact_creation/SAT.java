package cataclysm.contact_creation;

import java.util.Iterator;
import java.util.List;

import org.lwjgl.util.vector.Vector3f;

import cataclysm.Epsilons;
import cataclysm.contact_creation.ContactFeature.FeatureType;
import cataclysm.wrappers.ConvexHullWrapper;
import cataclysm.wrappers.ConvexHullWrapperFace;
import cataclysm.wrappers.ConvexHullWrapperHalfEdge;

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

	private static final ContactFeature onA = new ContactFeature();
	private static final ContactFeature onB = new ContactFeature();

	private static final boolean DEBUG = false;

	/**
	 * Teste l'intersection de deux enveloppes convexes.
	 * 
	 * @param hullA
	 * @param hullB
	 */
	static void overlapTest(ConvexHullWrapper hullA, ConvexHullWrapper hullB, ContactArea contact) {

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
		ConvexHullWrapperFace refFaceInA = referenceFace;

		if (DEBUG) {
			System.out.println("faceCheckDistanceA: " + faceCheckDistanceA);
		}

		if (faceCheckDistanceA >= 0.0f) {
			onA.setFrom(referenceFace);
			onB.clean();
			contact.rebuild(refFaceInA.getNormal(), faceCheckDistanceA, 0, onA, onB);
			contact.setNoCollision();
			return;
		}

		faceCheck(hullB, hullA);
		float faceCheckDistanceB = penetrationDepth;
		ConvexHullWrapperFace refFaceInB = referenceFace;

		if (DEBUG) {
			System.out.println("faceCheckDistanceB: " + faceCheckDistanceB);
		}

		if (faceCheckDistanceB >= 0.0f) {
			onA.clean();
			onB.setFrom(referenceFace);
			normal.set(refFaceInB.getNormal());
			normal.negate();
			contact.rebuild(normal, faceCheckDistanceB, 0, onA, onB);
			contact.setNoCollision();
			return;
		}

		float edgeCheckDistance = Float.NEGATIVE_INFINITY;
		// If the face check gives a good enough result, we skip the edge check
		// completely.
		if (contact.getFeatureOnA().getHalfedge() != null
				|| Math.max(faceCheckDistanceA, faceCheckDistanceB) < -Epsilons.ALLOWED_PENETRATION) {
			edgeCheck(hullA, hullB);
			edgeCheckDistance = penetrationDepth;
		}

		if (DEBUG) {
			System.out.println("edgeCheckDistance: " + edgeCheckDistance);
		}

		if (edgeCheckDistance > 0.0f) {
			onA.setFrom(contactEdgeA);
			onB.setFrom(contactEdgeB);
			contact.rebuild(normal, edgeCheckDistance, 0, onA, onB);
			contact.setNoCollision();
			return;
		}
		// }

		if (faceCheckDistanceA > edgeCheckDistance - Epsilons.ALLOWED_PENETRATION
				|| faceCheckDistanceB > edgeCheckDistance - Epsilons.ALLOWED_PENETRATION) {
			if (Math.abs(faceCheckDistanceA - faceCheckDistanceB) < Epsilons.ALLOWED_PENETRATION) {
				if (contact.getFeatureOnA().getFace() != null) {
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
				penetrationDepth = faceCheckDistanceA;
				referenceFace = refFaceInA;
				createFaceContact(contact, hullA, hullB, true);
			} else {
				penetrationDepth = faceCheckDistanceB;
				referenceFace = refFaceInB;
				createFaceContact(contact, hullA, hullB, false);
			}
		} else {
			penetrationDepth = edgeCheckDistance;
			createEdgeContact(contact, hullA, hullB);
		}

	}

	/**
	 * Si un test pr�c�dent a permi d'identifier un axe s�parateur, on le teste �
	 * nouveau pour permettre une sortie rapide de l'algo.
	 * 
	 * @param contact
	 * @return true si une sortie rapide de l'algo est possible.
	 */
	private static boolean failFastCheck(ConvexHullWrapper hullA, ConvexHullWrapper hullB, ContactArea contact) {
		if (DEBUG) {
			System.out.println("Fail fast check: previous distance:" + contact.getPenetrationDepth());
			System.out.println("ContactFeature sur A:" + contact.getFeatureOnA().getType());
			System.out.println("ContactFeature sur B:" + contact.getFeatureOnB().getType());
		}

		if (contact.getPenetrationDepth() >= 0) {
			ContactFeature onA = contact.getFeatureOnA();
			ContactFeature onB = contact.getFeatureOnB();

			if (onA.getType() == FeatureType.Face) {
				ConvexHullWrapperFace face = onA.getFace();
				hullB.getSupport(face.getNormal(), true, supportPoint);
				float distance = face.signedDistance(supportPoint);

				if (DEBUG) {
					System.out.println("pr�c�dente collision sur une face");
					System.out.println("referenceFace dans hullA : dist = " + distance);
				}

				if (distance > 0) {
					contact.setNoCollision();
					return true;
				}
			} else if (onB.getType() == FeatureType.Face) {
				ConvexHullWrapperFace face = onB.getFace();
				hullA.getSupport(face.getNormal(), true, supportPoint);
				float distance = face.signedDistance(supportPoint);

				if (DEBUG) {
					System.out.println("pr�c�dente collision sur une face");
					System.out.println("referenceFace dans hullB : dist = " + distance);
				}

				if (distance > 0) {
					contact.setNoCollision();
					return true;
				}
			} else if (onA.getType() == FeatureType.HalfEdge && onB.getType() == FeatureType.HalfEdge) {
				if (DEBUG) {
					System.out.println("pr�c�dente collision sur une ar�te");
				}

				ConvexHullWrapperHalfEdge edgeA = onA.getHalfedge();
				ConvexHullWrapperHalfEdge edgeB = onB.getHalfedge();
				Vector3f.sub(edgeA.getHead(), edgeA.getTail(), vecEdgeA);
				Vector3f.sub(edgeB.getHead(), edgeB.getTail(), vecEdgeB);

				if (!isMinkowskiFace(edgeA, vecEdgeA, edgeB, vecEdgeB)) {
					if (DEBUG) {
						System.out.println("Les ar�tes ne forment plus une face sur la diff�rence de Minkowski.");
					}
					return false;
				}

				centroidA.set(hullA.getCentroid());
				float distance = edgeDistance(edgeA, edgeB);

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
	 * Construit le polygone repr�sentant la surface de contact.
	 * 
	 * La face de r�f�rence est celle dont la normale est l'axe minimisant (en
	 * valeur absolue) la profondeur de p�n�tration.
	 * 
	 * @param contact
	 * @param hullA
	 * @param hullB
	 * @param refFaceInHullA indique si la face de reference est dans A ou dans B.
	 */
	private static void createFaceContact(ContactArea contact, ConvexHullWrapper hullA, ConvexHullWrapper hullB,
			boolean refFaceInHullA) {

		if (DEBUG) {
			System.out.println("createFaceContact");
		}

		ConvexHullWrapperFace incidentFace = (refFaceInHullA ? hullB : hullA)
				.getMostAntiParallelFace(referenceFace.getNormal());

		List<Vector3f> inputList = PolygonClipping.clipIncidentFaceAgainstReferenceFace(incidentFace, referenceFace);

		// On supprime les points au dessus du plan.
		normal.set(referenceFace.getNormal());
		for (Iterator<Vector3f> it = inputList.iterator(); it.hasNext();) {
			Vector3f vertex = it.next();
			float distance = referenceFace.signedDistance(vertex);
			if (distance > Epsilons.ALLOWED_PENETRATION) {
				it.remove();
			}
		}

		int contactCount = ReduceManifold.reduceManifold(inputList, normal, contact.contactPoints);

		// On projette les points sur le plan de la face de référence
		for (int i = 0; i < contactCount; i++) {
			Vector3f vertex = contact.contactPoints[i];
			float distance = referenceFace.signedDistance(vertex);
			vertex.x -= distance * normal.x;
			vertex.y -= distance * normal.y;
			vertex.z -= distance * normal.z;
			contact.penetrations[i] = distance;
		}

		if (!refFaceInHullA) {
			normal.negate();
			onA.clean();
			onB.setFrom(referenceFace);
		} else {
			onA.setFrom(referenceFace);
			onB.clean();
		}

		contact.rebuild(normal, penetrationDepth, contactCount, onA, onB);
	}

	//
	// Quelques variables pour createEdgeContact
	//
	private static final Vector3f A = new Vector3f();
	private static final Vector3f B = new Vector3f();
	private static final Vector3f C = new Vector3f();
	private static final Vector3f D = new Vector3f();

	private static final Vector3f AB = new Vector3f();
	private static final Vector3f CD = new Vector3f();
	private static final Vector3f AC = new Vector3f();

	/**
	 * Construit le point de contact, il minimise la distance entre les deux arr�tes
	 * de contact.
	 * 
	 * @param contact
	 * @param hullA
	 * @param hullB
	 */
	private static void createEdgeContact(ContactArea contact, ConvexHullWrapper hullA, ConvexHullWrapper hullB) {

		if (DEBUG) {
			System.out.println("createFaceContact");
		}

		A.set(contactEdgeA.getHead());
		B.set(contactEdgeA.getTail());

		C.set(contactEdgeB.getHead());
		D.set(contactEdgeB.getTail());

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

		contact.contactPoints[0].set(0.5f * (A.x + t * AB.x + C.x + s * CD.x), 0.5f * (A.y + t * AB.y + C.y + s * CD.y),
				0.5f * (A.z + t * AB.z + C.z + s * CD.z));
		contact.penetrations[0] = penetrationDepth;

		onA.setFrom(contactEdgeA);
		onB.setFrom(contactEdgeB);
		contact.rebuild(normal, penetrationDepth, 1, onA, onB);

	}

	// Quelques variables pour face check.
	private static float penetrationDepth;
	private static ConvexHullWrapperFace referenceFace;
	private static final Vector3f supportPoint = new Vector3f();

	/**
	 * Recherche un axe de s�paration entre les deux solides. Cet axe est la normale
	 * d'une face du solide 1.
	 * 
	 * @param hull1
	 * @param hull2
	 */
	private static void faceCheck(ConvexHullWrapper hull1, ConvexHullWrapper hull2) {
		penetrationDepth = Float.NEGATIVE_INFINITY;

		for (ConvexHullWrapperFace face : hull1.getFaces()) {

			hull2.getSupport(face.getNormal(), true, supportPoint);

			float distance = face.signedDistance(supportPoint);

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
	// Quelques variables pour edgeCheck
	//
	private static final Vector3f centroidA = new Vector3f();
	private static ConvexHullWrapperHalfEdge contactEdgeA;
	private static final Vector3f vecEdgeA = new Vector3f();
	private static ConvexHullWrapperHalfEdge contactEdgeB;
	private static final Vector3f vecEdgeB = new Vector3f();
	private static final Vector3f edgeAxEdgeB = new Vector3f();
	private static final Vector3f temp = new Vector3f();
	private static final Vector3f normal = new Vector3f();

	/**
	 * Recherche un axe de s�paration entre les deux solides. Cet axe est le produit
	 * vectoriel d'une arr�te de A avec une arr�te de B.
	 * 
	 * @param A
	 * @param B
	 */
	private static void edgeCheck(ConvexHullWrapper A, ConvexHullWrapper B) {
		penetrationDepth = Float.NEGATIVE_INFINITY;
		centroidA.set(A.getCentroid());

		ConvexHullWrapperHalfEdge[] edgesA = A.getEdges();
		ConvexHullWrapperHalfEdge[] edgesB = B.getEdges();

		for (int i = 0; i < edgesA.length; i += 2) {
			ConvexHullWrapperHalfEdge edgeA = edgesA[i];
			Vector3f.sub(edgeA.getHead(), edgeA.getTail(), vecEdgeA);

			for (int j = 0; j < edgesB.length; j += 2) {
				ConvexHullWrapperHalfEdge edgeB = edgesB[j];
				Vector3f.sub(edgeB.getHead(), edgeB.getTail(), vecEdgeB);

				if (!isMinkowskiFace(edgeA, vecEdgeA, edgeB, vecEdgeB)) {
					continue;
				}

				float distance = edgeDistance(edgeA, edgeB);

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
	 * Calcule la distance entre deux ar�tes.
	 * 
	 * Attention les variables suivantes doivent doit �tre � jour: centroidA,
	 * vecEdgeA, vecEdgeB
	 * 
	 * @param edgeA
	 * @param edgeB
	 * @return
	 */
	private static float edgeDistance(ConvexHullWrapperHalfEdge edgeA, ConvexHullWrapperHalfEdge edgeB) {
		Vector3f.cross(vecEdgeA, vecEdgeB, edgeAxEdgeB);
		float length2 = edgeAxEdgeB.lengthSquared();

		if (length2 < (1.0f - Epsilons.PARALLEL_LIMIT_2) * vecEdgeA.lengthSquared() * vecEdgeB.lengthSquared()) {// skip
																													// parallel
																													// edges.
			return Float.NEGATIVE_INFINITY;
		}
		float one_over_length = 1.0f / (float) Math.sqrt(length2);

		Vector3f.sub(edgeA.getTail(), centroidA, temp);
		if (Vector3f.dot(edgeAxEdgeB, temp) < 0) {
			one_over_length = -one_over_length;
		}

		edgeAxEdgeB.x *= one_over_length;
		edgeAxEdgeB.y *= one_over_length;
		edgeAxEdgeB.z *= one_over_length;

		Vector3f.sub(edgeB.getTail(), edgeA.getTail(), temp);
		float distance = Vector3f.dot(edgeAxEdgeB, temp);

		return distance;
	}

	/**
	 * Teste si le produit vectoriel d'une combinaison de deux arr�tes forme un axe
	 * de s�paration possible.
	 * 
	 * @param edgeA
	 * @param vecEdgeA
	 * @param edgeB
	 * @param vecEdgeB
	 * @return true si la s�paration est possible pour ce couple d'arr�te.
	 */
	private static boolean isMinkowskiFace(ConvexHullWrapperHalfEdge edgeA, Vector3f vecEdgeA,
			ConvexHullWrapperHalfEdge edgeB, Vector3f vecEdgeB) {

		float CBA = Vector3f.dot(edgeB.getFaceNormal(), vecEdgeA);
		float DBA = Vector3f.dot(edgeB.getAdjacentFaceNormal(), vecEdgeA);
		float ADC = -Vector3f.dot(edgeA.getFaceNormal(), vecEdgeB);
		float BDC = -Vector3f.dot(edgeA.getAdjacentFaceNormal(), vecEdgeB);

		return CBA * DBA < 0 && ADC * BDC < 0 && CBA * BDC > 0;
	}

}
