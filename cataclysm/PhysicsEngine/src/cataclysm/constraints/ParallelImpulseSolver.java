package cataclysm.constraints;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.function.Consumer;

import cataclysm.Epsilons;
import cataclysm.contact_creation.AbstractDoubleBodyContact;
import cataclysm.contact_creation.AbstractSingleBodyContact;
import math.vector.Vector3f;

/**
 * Applique des impulsions sur les corps jusqu'à satisfaction des contraintes et
 * des contacts.
 * 
 * @author Briac
 *
 */
public class ParallelImpulseSolver implements ConstraintSolver {

	private static final boolean DEBUG = false;

	private static class WorkerThread extends Thread {

		private final Object monitor = new Object();
		private final int threadIndex;
		private final CyclicBarrier allTerminated;
		private boolean shouldExit = false;
		private Consumer<WorkerThread> work;

		private int doubleBodyContacts = 0;
		private int singleBodyContacts = 0;
		private int constraints = 0;

		public WorkerThread(int threadIndex, CyclicBarrier allTerminated) {
			this.threadIndex = threadIndex;
			this.allTerminated = allTerminated;
			setDaemon(true);
			setName("ParallelImpulseSolver " + threadIndex);
			setDefaultUncaughtExceptionHandler(new UncaughtExceptionHandler() {

				@Override
				public void uncaughtException(Thread t, Throwable e) {
					System.err.println("Exception in thread i=" + threadIndex);
					e.printStackTrace();
				}
			});
		}

		@Override
		public void run() {

			while (!shouldExit) {
				waitLoop();
				if (shouldExit) {
					break;
				}
				work.accept(this);
				work = null;
				try {
					allTerminated.await();
				} catch (InterruptedException | BrokenBarrierException e) {
					e.printStackTrace();
				}
			}

		}

		private void waitLoop() {
			while (work == null && !shouldExit) {
				synchronized (monitor) {
					try {
						monitor.wait();
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
		}

		public void setWorkAndWake(Consumer<WorkerThread> work) {
			this.work = work;
			synchronized (monitor) {
				monitor.notify();
			}
		}

		public void shouldExit() {
			shouldExit = true;
			monitor.notify();
		}

	}

	private final int threadCount;

	private final List<WorkerThread> threads;
	private final CyclicBarrier allTerminated;
	private final CyclicBarrier groupBarrier;

	public ParallelImpulseSolver(int threadCount) {
		this.threadCount = threadCount;
		threads = new ArrayList<WorkerThread>(threadCount);
		allTerminated = new CyclicBarrier(threadCount + 1);
		groupBarrier = new CyclicBarrier(threadCount);

		for (int i = 0; i < threadCount; i++) {
			WorkerThread worker = new WorkerThread(i, allTerminated);
			threads.add(worker);
			worker.start();
		}

	}

	@Override
	public void solve(List<AbstractSingleBodyContact> activeMeshContacts,
			List<AbstractDoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints, float timeStep,
			int MAX_ITERATIONS_POSITION, int MAX_ITERATIONS_VELOCITY) {

		for (int i = 0; i < threadCount; i++) {
			final int threadIndex = i;
			final List<AbstractSingleBodyContact> meshes = buildSubList(activeMeshContacts, threadIndex, threadCount);
			final List<AbstractDoubleBodyContact> bodies = buildSubList(activeBodyContacts, threadIndex, threadCount);
			final List<AbstractConstraint> constraintsSubList;
			
			if(threadIndex == 0) {
				constraintsSubList = constraints;
			}else {
				constraintsSubList = Collections.emptyList();
			}
			
			Consumer<WorkerThread> work = (worker) -> solve(meshes, bodies, constraintsSubList, timeStep,
					MAX_ITERATIONS_POSITION, MAX_ITERATIONS_VELOCITY, worker);

			threads.get(i).setWorkAndWake(work);
		}

		try {
			allTerminated.await();
		} catch (InterruptedException | BrokenBarrierException e) {
			e.printStackTrace();
		}
	}

	private void solve(List<AbstractSingleBodyContact> activeMeshContacts,
			List<AbstractDoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints, float timeStep,
			int MAX_ITERATIONS_POSITION, int MAX_ITERATIONS_VELOCITY, WorkerThread worker) {

		// the velocity must be solved first, since the position correction needs data
		// computed during the velocity step.
		for (int iteration = 0; iteration < MAX_ITERATIONS_VELOCITY; iteration++) {
			solveVelocity(activeMeshContacts, activeBodyContacts, constraints, timeStep, iteration);
			try {
				groupBarrier.await();
			} catch (InterruptedException | BrokenBarrierException e) {
				e.printStackTrace();
			}
		}

		for (int iteration = 0; iteration < MAX_ITERATIONS_POSITION; iteration++) {
			solvePosition(activeMeshContacts, activeBodyContacts, constraints, timeStep, iteration == 0);
			try {
				groupBarrier.await();
			} catch (InterruptedException | BrokenBarrierException e) {
				e.printStackTrace();
			}
		}

		worker.doubleBodyContacts = activeBodyContacts.size();
		worker.singleBodyContacts = activeMeshContacts.size();
		worker.constraints = constraints.size();

		if (DEBUG)
			System.err.println(" ID: " + worker.threadIndex + " DoubleBodies: " + worker.doubleBodyContacts
					+ " SingleBodies: " + worker.singleBodyContacts + " Constraints: " + worker.constraints);
	}

	/**
	 * Applique des impulsions pour corriger les erreurs de vitesse.
	 * 
	 * @param activeMeshContacts
	 * @param activeBodyContacts
	 * @param constraints
	 * @param timeStep
	 */
	private void solveVelocity(List<AbstractSingleBodyContact> activeMeshContacts,
			List<AbstractDoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints, float timeStep,
			int iteration) {

		final Vector3f temp = new Vector3f();

		if (iteration == 0) {

			for (AbstractDoubleBodyContact contact : activeBodyContacts) {
				contact.velocityStart();
			}

			for (AbstractSingleBodyContact contact : activeMeshContacts) {
				contact.velocityStart();
			}

			for (AbstractDoubleBodyContact contact : activeBodyContacts) {
				if (Epsilons.WARM_START) {
					contact.warmStart();
				} else {
					contact.resetImpulses();
				}
				contact.solveVelocity();
			}

			for (AbstractConstraint constraint : constraints) {
				constraint.solveVelocity(true, timeStep, temp);
			}

			for (AbstractSingleBodyContact contact : activeMeshContacts) {
				if (Epsilons.WARM_START) {
					contact.warmStart();
				} else {
					contact.resetImpulses();
				}
				contact.solveVelocity();
			}

		} else {
			for (AbstractDoubleBodyContact contact : activeBodyContacts) {
				contact.solveVelocity();
			}

			for (AbstractConstraint constraint : constraints) {
				constraint.solveVelocity(false, timeStep, temp);
			}

			for (AbstractSingleBodyContact contact : activeMeshContacts) {
				contact.solveVelocity();
			}
		}

	}

	/**
	 * Applique des impulsions pour corriger les erreurs de position. Ces impulsions
	 * ne modifient pas la vitesse des solides.
	 * 
	 * @param activeMeshContacts
	 * @param activeBodyContacts
	 * @param constraints
	 * @param timeStep
	 */
	private void solvePosition(List<AbstractSingleBodyContact> activeMeshContacts,
			List<AbstractDoubleBodyContact> activeBodyContacts, List<AbstractConstraint> constraints, float timeStep,
			boolean firstIteration) {

		final Vector3f temp = new Vector3f();

		if (firstIteration) {
			for (AbstractDoubleBodyContact contact : activeBodyContacts) {
				contact.positionStart(timeStep);
				contact.solvePosition();
			}

			for (AbstractConstraint constraint : constraints) {
				constraint.solvePosition(true, timeStep, temp);
			}

			for (AbstractSingleBodyContact contact : activeMeshContacts) {
				contact.positionStart(timeStep);
				contact.solvePosition();
			}
		} else {
			for (AbstractDoubleBodyContact contact : activeBodyContacts) {
				contact.solvePosition();
			}

			for (AbstractConstraint constraint : constraints) {
				constraint.solvePosition(false, timeStep, temp);
			}

			for (AbstractSingleBodyContact contact : activeMeshContacts) {
				contact.solvePosition();
			}
		}

	}

	private static <E> List<E> buildSubList(List<E> list, int threadIndex, int threadCount) {
		long slice = arraySlice(list.size(), threadIndex, threadCount);
		int start = (int) (slice >> 32);
		int stop = (int) slice;
		return list.subList(start, stop);
	}

	private static long arraySlice(int arrayLength, int threadIndex, int threadCount) {
		int size = arrayLength / threadCount;
		int start = size * threadIndex;
		int stop = start + size;
		if (threadIndex + 1 == threadCount) {
			stop = arrayLength;
		}
		return ((long) start) << 32 | stop;
	}

	public void cleanUp() {
		for (int i = 0; i < threadCount; i++) {
			WorkerThread thread = threads.get(i);
			thread.shouldExit();
		}
		threads.clear();
	}

}
