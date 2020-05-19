package cataclysm.contact_creation;

import cataclysm.wrappers.ConvexHullWrapper;

/**
 * Permet de tester la collision entre deux enveloppe convexe par l'algorithme SAT.
 * @author Briac
 *
 */
class CollideHulls {
	
	private final SAT sat = new SAT();

	void test(ConvexHullWrapper A, ConvexHullWrapper B, ContactZone contact) {
		sat.overlapTest(A, B, contact);
	}

}
