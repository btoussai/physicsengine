package cataclysm.parallel;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;

/**
 * Represents a thread part of a {@link PhysicsWorkerPool}.
 * 
 * @author Briac
 *
 */
public class PhysicsWorkerThread extends Thread {

	private final int threadIndex;
	private final int threadCount;

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
	 * A queue in which the work to be done is stored temporarily
	 */
	private final ArrayBlockingQueue<PhysicsWork> queue;

	public PhysicsWorkerThread(int threadIndex, int threadCount, CyclicBarrier waitForTermination,
			CyclicBarrier waitForGroup, int maxWorks) {
		this.queue = new ArrayBlockingQueue<PhysicsWork>(maxWorks);
		this.threadIndex = threadIndex;
		this.threadCount = threadCount;
		this.waitForTermination = waitForTermination;
		this.waitForGroup = waitForGroup;
		setDefaultUncaughtExceptionHandler((t, e) -> this.uncaughtException(t, e));
		setDaemon(true);
		setName("PhysicsWorkerThread " + threadIndex);
	}

	private void uncaughtException(Thread t, Throwable e) {
		System.err.println("Exception in thread " + getName());
		e.printStackTrace();
		System.exit(0);
	}

	@Override
	public void run() {

		while (true) {
			final PhysicsWork w;
			try {
				w = queue.take();
			} catch (InterruptedException e) {
				e.printStackTrace();
				break;
			}
			//System.out.println(this.toString() + ": " + w.toString());
			w.run(this);
		}

	}

	void scheduleWork(PhysicsWork w) {
		queue.add(w);
	}

	public int getThreadIndex() {
		return threadIndex;
	}

	/**
	 * Wait for the other threads of the {@link PhysicsWorkerPool} and for the main
	 * thread to call {@link PhysicsWorkerPool#waitForTaskTermination()}.
	 */
	public void waitForTermination() {
		try {
			waitForTermination.await();
		} catch (InterruptedException | BrokenBarrierException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Wait for the other threads of the {@link PhysicsWorkerPool}.
	 */
	public void waitForGroup() {
		try {
			waitForGroup.await();
		} catch (InterruptedException | BrokenBarrierException e) {
			e.printStackTrace();
		}
	}

	public CyclicBarrier getWaitForTermination() {
		return waitForTermination;
	}

	public CyclicBarrier getWaitForGroup() {
		return waitForGroup;
	}

	public int getThreadCount() {
		return threadCount;
	}

	@Override
	public String toString() {
		return "PhysicsWorkerThread " + threadIndex;
	}

}
