package cataclysm;

import java.util.List;

import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.constraints.AbstractConstraint;
import cataclysm.constraints.SequentialImpulseSolver;
import cataclysm.record.PhysicsPlayer;
import cataclysm.wrappers.RigidBodyManager;

/**
 * Permet de simuler les intéractions entre les objets.
 * 
 * @author Briac
 *
 */
final class PhysicsEngine extends AbstractPhysicsEngine{

	/**
	 * Instancie un moteur physique. Il s'occupe de la mise à jour des objets.
	 * 
	 * @param params
	 */
	PhysicsEngine(PhysicsWorld world) {
		super(world, new SequentialImpulseSolver());
	}

	@Override
	protected void update(RigidBodyManager bodies, StaticMeshManager meshes, List<AbstractConstraint> constraints,
			PhysicsStats stats) {

		if (world.getRecordPlayers() != null) {
			stats.physicsPlayers.start();
			for (PhysicsPlayer player : world.getRecordPlayers()) {
				player.step(meshes, bodies);
			}
			stats.physicsPlayers.stop();
		}
		
		meshes.update();

		float timeStep = params.getTimeStep();
		boolean gyroscopicIntegration = params.useGyroscopicIntegration();

		applyForces(bodies.getElements(), timeStep);

		stats.broadAndNarrowphase.start();
		bodies.update();
		stats.broadAndNarrowphase.stop();

		stats.reset(bodies.size(), meshes.size(), constraints.size(), 1);

		stats.constraintSolver.start();
		solver.solve(bodies.getMeshContacts(), bodies.getBodyContacts(), constraints, timeStep,
				params.getMaxIterationsPosition(), params.getMaxIterationVelocity());
		stats.constraintSolver.stop();

		stats.velocityIntegration.start();
		integrateVelocity(bodies.getElements(), timeStep, gyroscopicIntegration);
		stats.velocityIntegration.stop();

		if (world.getActiveRecord() != null) {
			world.getUpdateStats().physicsRecorder.start();
			world.getActiveRecord().getCurrentFrame().fillBodiesStates(world);
			world.getUpdateStats().physicsRecorder.pause();
		}
	}

}
