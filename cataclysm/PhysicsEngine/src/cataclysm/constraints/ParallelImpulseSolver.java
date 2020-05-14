package cataclysm.constraints;

import java.util.ArrayList;
import java.util.Arrays;
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

		private int doubleBodyContactsLength = 0;
		private int[] doubleBodyContacts = new int[1000];

		private int constraintsLength = 0;
		private int[] constraints = new int[200];

		private int singleBodyContacts = 0;

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

		/**
		 * Fetches the i-th double body contact index that this thread is responsible of
		 * 
		 * @param i
		 * @return
		 */
		public int getDoubleBodyContact(int i) {
			if (i + 1 > doubleBodyContactsLength) {
				throw new IndexOutOfBoundsException(
						"Total bodyContacts: " + doubleBodyContactsLength + ", asked: " + i);
			}
			return doubleBodyContacts[i];
		}

		public int getDoubleBodyContactCount() {
			return doubleBodyContactsLength;
		}

		public void resetDoubleBodyContactCount() {
			doubleBodyContactsLength = 0;
		}

		public void addDoubleBodyContact(int i) {
			doubleBodyContactsLength++;
			if (doubleBodyContacts.length < doubleBodyContactsLength) {
				doubleBodyContacts = Arrays.copyOf(doubleBodyContacts, doubleBodyContactsLength * 2);
			}
			doubleBodyContacts[doubleBodyContactsLength - 1] = i;
		}

		/**
		 * Fetches the i-th constraint index that this thread is responsible of
		 * 
		 * @param i
		 * @return
		 */
		public int getConstraint(int i) {
			if (i + 1 > constraintsLength) {
				throw new IndexOutOfBoundsException("Total bodyContacts: " + constraintsLength + ", asked: " + i);
			}
			return constraints[i];
		}

		public int getConstraintCount() {
			return constraintsLength;
		}

		public void resetConstraintCount() {
			constraintsLength = 0;
		}

		public void addConstraint(int i) {
			constraintsLength++;
			if (constraints.length < constraintsLength) {
				constraints = Arrays.copyOf(doubleBodyContacts, constraintsLength * 2);
			}
			constraints[constraintsLength - 1] = i;
		}
	}

	private final Vector3f temp = new Vector3f();
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

		Consumer<WorkerThread> work = (worker) -> solve(activeMeshContacts, activeBodyContacts, constraints, timeStep,
				MAX_ITERATIONS_POSITION, MAX_ITERATIONS_VELOCITY, worker);

		for (int i = 0; i < threadCount; i++) {
			WorkerThread thread = threads.get(i);
			thread.setWorkAndWake(work);
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
			solveVelocity(activeMeshContacts, activeBodyContacts, constraints, timeStep, iteration, worker);
			try {
				groupBarrier.await();
			} catch (InterruptedException | BrokenBarrierException e) {
				e.printStackTrace();
			}
		}

		for (int iteration = 0; iteration < MAX_ITERATIONS_POSITION; iteration++) {
			solvePosition(activeMeshContacts, activeBodyContacts, constraints, timeStep, iteration == 0, worker);
			try {
				groupBarrier.await();
			} catch (InterruptedException | BrokenBarrierException e) {
				e.printStackTrace();
			}
		}

		if(DEBUG)
		System.err.println(" ID: " + worker.threadIndex + " DoubleBodies: " + worker.getDoubleBodyContactCount()
				+ " SingleBodies: " + worker.singleBodyContacts + " Constraints: " + worker.getConstraintCount());
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
			int iteration, WorkerThread worker) {

		int threadIndex = worker.threadIndex;
		int start, stop;
		long slice;

		if (iteration == 0) {

			worker.resetDoubleBodyContactCount();
			for (int i = 0; i < activeBodyContacts.size(); i++) {
				AbstractDoubleBodyContact contact = activeBodyContacts.get(i);
				if (canDispatch(contact, threadIndex, threadCount)) {
					worker.addDoubleBodyContact(i);
					contact.velocityStart();
				}
			}

			slice = arraySlice(activeMeshContacts.size(), threadIndex);
			start = (int) (slice >> 32);
			stop = (int) slice;
			for (int i = start; i < stop; i++) {
				AbstractSingleBodyContact contact = activeMeshContacts.get(i);
				contact.velocityStart();
			}
			worker.singleBodyContacts = stop - start;

			for (int i = 0; i < worker.getDoubleBodyContactCount(); i++) {
				AbstractDoubleBodyContact contact = activeBodyContacts.get(worker.getDoubleBodyContact(i));
				if (Epsilons.WARM_START) {
					contact.warmStart();
				} else {
					contact.resetImpulses();
				}
				contact.solveVelocity();
			}

			worker.resetConstraintCount();
			for (int i = 0; i < constraints.size(); i++) {
				AbstractConstraint constraint = constraints.get(i);
				if (canDispatch(constraint, threadIndex, threadCount)) {
					worker.addConstraint(i);
					constraint.solveVelocity(true, timeStep, temp);
				}
			}

			slice = arraySlice(activeMeshContacts.size(), threadIndex);
			start = (int) (slice >> 32);
			stop = (int) slice;
			for (int i = start; i < stop; i++) {
				AbstractSingleBodyContact contact = activeMeshContacts.get(i);
				if (Epsilons.WARM_START) {
					contact.warmStart();
				} else {
					contact.resetImpulses();
				}
				contact.solveVelocity();
			}

		} else {
			for (int i = 0; i < worker.getDoubleBodyContactCount(); i++) {
				AbstractDoubleBodyContact contact = activeBodyContacts.get(worker.getDoubleBodyContact(i));
				contact.solveVelocity();
			}

			for (int i = 0; i < worker.getConstraintCount(); i++) {
				AbstractConstraint constraint = constraints.get(worker.getConstraint(i));
				constraint.solveVelocity(false, timeStep, temp);
			}

			slice = arraySlice(activeMeshContacts.size(), threadIndex);
			start = (int) (slice >> 32);
			stop = (int) slice;
			for (int i = start; i < stop; i++) {
				AbstractSingleBodyContact contact = activeMeshContacts.get(i);
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
			boolean firstIteration, WorkerThread worker) {

		int threadIndex = worker.threadIndex;
		int start, stop;
		long slice;

		if (firstIteration) {
			for (int i = 0; i < worker.getDoubleBodyContactCount(); i++) {
				AbstractDoubleBodyContact contact = activeBodyContacts.get(worker.getDoubleBodyContact(i));
				contact.positionStart(timeStep);
				contact.solvePosition();
			}

			for (int i = 0; i < worker.getConstraintCount(); i++) {
				AbstractConstraint constraint = constraints.get(worker.getConstraint(i));
				constraint.solvePosition(true, timeStep, temp);
			}

			slice = arraySlice(activeMeshContacts.size(), threadIndex);
			start = (int) (slice >> 32);
			stop = (int) slice;
			for (int i = start; i < stop; i++) {
				AbstractSingleBodyContact contact = activeMeshContacts.get(i);
				contact.positionStart(timeStep);
				contact.solvePosition();
			}
		} else {
			for (int i = 0; i < worker.getDoubleBodyContactCount(); i++) {
				AbstractDoubleBodyContact contact = activeBodyContacts.get(worker.getDoubleBodyContact(i));
				contact.solvePosition();
			}

			for (int i = 0; i < worker.getConstraintCount(); i++) {
				AbstractConstraint constraint = constraints.get(worker.getConstraint(i));
				constraint.solvePosition(false, timeStep, temp);
			}

			slice = arraySlice(activeMeshContacts.size(), threadIndex);
			start = (int) (slice >> 32);
			stop = (int) slice;
			for (int i = start; i < stop; i++) {
				AbstractSingleBodyContact contact = activeMeshContacts.get(i);
				contact.solvePosition();
			}
		}

	}

	private boolean canDispatch(AbstractDoubleBodyContact contact, int thread_index, int thread_count) {
		return canDispatch(contact.getWrapperA().getBody().getID(), contact.getWrapperB().getBody().getID(),
				thread_index, thread_count);
	}

	private boolean canDispatch(AbstractConstraint constraint, int thread_index, int thread_count) {
		/*
		 * AnchorPoint pA = constraint.getPointA(); AnchorPoint pB =
		 * constraint.getPointB();
		 * 
		 * if (pA.isStatic()) { long b = pB.getBody().getID(); return canDispatch(b, b,
		 * thread_index, thread_count); } else if (pB.isStatic()) { long a =
		 * pA.getBody().getID(); return canDispatch(a, a, thread_index, thread_count); }
		 * else { long a = pA.getBody().getID(); long b = pB.getBody().getID(); return
		 * canDispatch(a, b, thread_index, thread_count); }
		 */
		return thread_index + 1 == thread_count;
	}

	private boolean canDispatch(long a, long b, int thread_index, int thread_count) {
		int modA, modB;
		switch (thread_count) {
		case 0:
			throw new IllegalArgumentException("Invalid thread count: " + thread_count);
		case 1:
			return true;
		case 2:
			modA = (int) (a % 3);
			modB = (int) (b % 3);
			if (modA == 0 || modB == 0) {
				return thread_index == 0;
			}
			return thread_index == 1;
		case 3:
			modA = (int) (a % 5);
			modB = (int) (b % 5);
			if (modA == 0 || modB == 0) {
				return thread_index == 0;
			}
			if (modA == 1 || modB == 1) {
				return thread_index == 1;
			}
			return thread_index == 2;
		case 4:
			modA = (int) (a % 7);
			modB = (int) (b % 7);
			if (modA == 0 || modB == 0) {
				return thread_index == 0;
			}
			if (modA == 1 || modB == 1) {
				return thread_index == 1;
			}
			if (modA == 2 || modB == 2) {
				return thread_index == 2;
			}
			return thread_index == 3;
		default:
			throw new IllegalArgumentException("Invalid thread count: " + thread_count);
		}

	}

	private long arraySlice(int arrayLength, int threadIndex) {
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
