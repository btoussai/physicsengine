/**
 * 
 */
/**
 * @author Briac Toussaint
 *
 */
module org.briac.PhysicsEngine {
	exports cataclysm.constraints;
	exports cataclysm.broadphase.staticmeshes;
	exports cataclysm.parallel;
	exports cataclysm.contact_creation;
	exports cataclysm.datastructures;
	exports cataclysm.wrappers;
	exports cataclysm;
	exports cataclysm.quickHull;
	exports cataclysm.record;
	exports cataclysm.integrators;
	exports cataclysm.broadphase;

	requires transitive org.briac.MathLib;
}