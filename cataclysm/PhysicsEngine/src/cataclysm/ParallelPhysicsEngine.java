package cataclysm;

import java.util.ArrayList;
import java.util.List;

import cataclysm.broadphase.staticmeshes.StaticMeshManager;
import cataclysm.constraints.AbstractConstraint;
import cataclysm.constraints.ParallelImpulseSolver;
import cataclysm.parallel.PhysicsWork;
import cataclysm.parallel.PhysicsWorkerPool;
import cataclysm.parallel.PhysicsWorkerThread;
import cataclysm.record.PhysicsPlayer;
import cataclysm.wrappers.RigidBody;
import cataclysm.wrappers.RigidBodyManager;

/**
 * Permet de simuler les intéractions entre les objets.
 * 
 * @author Briac
 *
 */
final class ParallelPhysicsEngine extends AbstractPhysicsEngine{

	private final PhysicsWorkerPool workers;

	/**
	 * Instancie un moteur physique. Il s'occupe de la mise à jour des objets.
	 * 
	 * @param params
	 */
	ParallelPhysicsEngine(PhysicsWorld world, int threads) {
		super(world, new ParallelImpulseSolver(new PhysicsWorkerPool(threads)));
		workers = ((ParallelImpulseSolver)super.solver).getWorkers();
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

		meshes.parallelUpdate(workers);

		float timeStep = params.getTimeStep();
		boolean gyroscopicIntegration = params.useGyroscopicIntegration();

		applyForces(bodies, timeStep);

		bodies.parallelUpdate(workers);

		stats.reset(bodies.size(), meshes.size(), constraints.size(), workers.getThreadCount());

		solver.solve(bodies.getMeshContacts(), bodies.getBodyContacts(), constraints, timeStep, params.getMaxIterationsPosition(),
				params.getMaxIterationVelocity());

		integrateVelocity(bodies, timeStep, gyroscopicIntegration);

		if (world.getActiveRecord() != null) {
			world.getUpdateStats().physicsRecorder.start();
			world.getActiveRecord().getCurrentFrame().fillBodiesStates(world);
			world.getUpdateStats().physicsRecorder.pause();
		}
	}

	private void applyForces(RigidBodyManager bodies, float timeStep) {
		List<PhysicsWork> tasks = new ArrayList<PhysicsWork>();
		for (int i = 0; i < workers.getThreadCount(); i++) {
			final List<RigidBody> slice = workers.buildSubList(bodies.getElements(), i);
			tasks.add(new PhysicsWork() {

				@Override
				public void run(PhysicsWorkerThread physicsWorkerThread) {
					applyForces(slice, timeStep);
					physicsWorkerThread.waitForGroup();
				}

			});
		}
		forceInegrator.prepare();
		workers.scheduleWork(tasks);
	}

	private void integrateVelocity(RigidBodyManager bodies, float timeStep, boolean gyroscopicIntegration) {
		List<PhysicsWork> tasks = new ArrayList<PhysicsWork>();
		for (int i = 0; i < workers.getThreadCount(); i++) {
			final List<RigidBody> slice = workers.buildSubList(bodies.getElements(), i);
			tasks.add(new PhysicsWork() {

				@Override
				public void run(PhysicsWorkerThread physicsWorkerThread) {
					integrateVelocity(slice, timeStep, gyroscopicIntegration);
					physicsWorkerThread.waitForTermination();
				}

			});
		}
		workers.scheduleWork(tasks);
		workers.waitForTaskTermination();
	}

	public PhysicsWorkerPool getWorkers() {
		return workers;
	}


}
