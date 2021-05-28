package cataclysm.parallel;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;

/**
 * Represents a set of threads.
 * 
 * @author Briac
 *
 */
public class PhysicsWorkerPool {

	/**
	 * A runnable executed when the syncronizing barrier is tripped. The action is
	 * repeated a set number of times.
	 * 
	 * @author Briac
	 *
	 */
	private static class RepeatableBarrierAction {
		private int repetitions;
		private Runnable action;

		public RepeatableBarrierAction(int repetitions, Runnable action) {
			if (repetitions < 1) {
				throw new IllegalArgumentException("Invalid action repetition count, must be >= 1, got " + repetitions);
			}
			if (action == null) {
				throw new NullPointerException("The runnable action cannot be null");
			}
			this.repetitions = repetitions;
			this.action = action;
		}
	}

	/**
	 * The threads in the set
	 */
	private final List<PhysicsWorkerThread> threads;

	/**
	 * A barrier meant for synchronization between the threads of the group and the
	 * caller
	 */
	private final CyclicBarrier waitForTermination;

	/**
	 * A barrier meant for synchronization between the threads of the group.
	 */
	private final CyclicBarrier waitForGroup;

	/**
	 * A queue containing the next barrierActions that will be executed when the
	 * waiting barrier is tripped
	 */
	private final ArrayBlockingQueue<RepeatableBarrierAction> barrierActions;

	/**
	 * Creates a new thread group containing threadCount threads.
	 * 
	 * @param threadCount
	 * @param maxWorks    The max number of instances of work that can be scheduled
	 */
	public PhysicsWorkerPool(int threadCount, int maxWorks) {
		if (threadCount < 1) {
			throw new IllegalArgumentException("Invalid thread count, should be >= 1, got: " + threadCount);
		}
		if (maxWorks < 1) {
			throw new IllegalArgumentException("Invalid max work, should be >= 1, got: " + maxWorks);
		}
		barrierActions = new ArrayBlockingQueue<>(maxWorks);
		waitForTermination = new CyclicBarrier(threadCount + 1, this::barrierAction);
		waitForGroup = new CyclicBarrier(threadCount, this::barrierAction);
		threads = new ArrayList<>(threadCount);
		for (int i = 0; i < threadCount; i++) {
			PhysicsWorkerThread thread = new PhysicsWorkerThread(i, threadCount, waitForTermination, waitForGroup,
					maxWorks);
			threads.add(thread);
			thread.start();
		}
	}

	private void defaultAction() {
	}

	private void barrierAction() {
		RepeatableBarrierAction action = barrierActions.peek();
		if (action == null) {
			throw new IllegalStateException("Mismatched synchronizedCount detected while scheduling a PhysicsWork");
		}
		action.action.run();
		if (--action.repetitions == 0) {
			barrierActions.remove();
		}
	}

	/**
	 * Schedules a task for each thread in the group.
	 * 
	 * @param work
	 * @param taskName          The name of the task
	 * @param synchronizedCount The number of times the threads will have to
	 *                          synchronize and wait for each other inside the work.
	 */
	public void scheduleWork(List<PhysicsWork> work, String taskName, int synchronizedCount) {
		this.scheduleWork(work, taskName, this::defaultAction, synchronizedCount);
	}

	/**
	 * Schedules a task for each thread in the group
	 * 
	 * @param work
	 * @param taskName          The name of the task
	 * @param barrierAction     An action to be executed when the waiting barrier is
	 *                          tripped
	 * @param synchronizedCount The number of times the threads will have to
	 *                          synchronize and wait for each other inside the work
	 */
	public void scheduleWork(List<PhysicsWork> work, String taskName, Runnable barrierAction, int synchronizedCount) {
		if (work.size() != threads.size()) {
			throw new IllegalArgumentException(
					"There should be one task per worker thread, got " + work.size() + " instead of " + threads.size());
		}

		if (!barrierActions.offer(new RepeatableBarrierAction(synchronizedCount, barrierAction))) {
			throw new IllegalStateException("Too much work scheduled");
		}

		for (int i = 0; i < work.size(); i++) {
			PhysicsWork w = work.get(i);
			w.name = taskName + " worker " + i;
			threads.get(i).scheduleWork(w);
		}
	}

	/**
	 * Waits for the termination of a task requiring the caller to synchronize with
	 * all the threads in the group when the task is finished.
	 * 
	 */
	public void waitForTaskTermination() {
		try {
			waitForTermination.await();
		} catch (InterruptedException | BrokenBarrierException e) {
			e.printStackTrace();
		}
	}

	public int getThreadCount() {
		return threads.size();
	}

	public <E> List<E> buildSubList(List<E> list, int threadIndex) {
		int size = list.size() / getThreadCount();
		int start = size * threadIndex;
		int stop = start + size;
		if (threadIndex + 1 == getThreadCount()) {
			stop = list.size();
		}
		return list.subList(start, stop);
	}
}
