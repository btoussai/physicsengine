package cataclysm.contact_creation;

import cataclysm.wrappers.ConvexHullWrapper;

/**
 * Permet de tester la collision entre deux enveloppe convexe par l'algorithme SAT.
 * @author Briac
 *
 */
class CollideHulls {

	static void test(ConvexHullWrapper A, ConvexHullWrapper B, ContactArea contact) {
		SAT.overlapTest(A, B, contact);
	}

}
