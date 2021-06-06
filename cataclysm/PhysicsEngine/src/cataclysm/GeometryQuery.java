package cataclysm;

import java.util.Set;

import cataclysm.annotations.Parallelizable;
import cataclysm.broadphase.AABB;
import cataclysm.broadphase.staticmeshes.Triangle;
import cataclysm.wrappers.Wrapper;

public interface GeometryQuery {

	/**
	 * Performs a ray test against geometry in the simulation.
	 * 
	 * @param test
	 */
	@Parallelizable
	public void rayTest(RayTest test);

	/**
	 * Performs a box test against static meshes All primitives intersecting the box
	 * will be added in the set. The set should be cleared before calling the
	 * function.
	 * 
	 * @param box
	 * @param set
	 */
	@Parallelizable
	public void boxTriangleQuery(AABB box, Set<Triangle> set);

	/**
	 * Performs a box test against wrappers All primitives intersecting the box will
	 * be added in the set. The set should be cleared before calling the function.
	 * 
	 * @param box
	 * @param set
	 */
	@Parallelizable
	public void boxWrapperQuery(AABB box, Set<Wrapper> set);

}
