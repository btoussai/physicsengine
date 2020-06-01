package cataclysm.parallel;

import java.util.ArrayList;
import java.util.List;
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
	 * Creates a new thread group containing threadCount threads.
	 * 
	 * @param threadCount
	 */
	public PhysicsWorkerPool(int threadCount) {
		if (threadCount < 2) {
			throw new IllegalArgumentException("Invalid thread count, should be >= 2, got: " + threadCount);
		}
		waitForTermination = new CyclicBarrier(threadCount + 1);
		waitForGroup = new CyclicBarrier(threadCount);
		threads = new ArrayList<>(threadCount);
		for (int i = 0; i < threadCount; i++) {
			PhysicsWorkerThread thread = new PhysicsWorkerThread(i, threadCount, waitForTermination, waitForGroup);
			threads.add(thread);
			thread.start();
		}
	}

	/**
	 * Schedules a task for each thread in the group
	 * 
	 * @param work
	 */
	public void scheduleWork(List<PhysicsWork> work) {
		if (work.size() != threads.size()) {
			throw new IllegalArgumentException(
					"There should be one task per worker thread, got " + work.size() + " instead of " + threads.size());
		}
		for (int i = 0; i < work.size(); i++) {
			try {
				threads.get(i).scheduleWork(work.get(i));
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
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
