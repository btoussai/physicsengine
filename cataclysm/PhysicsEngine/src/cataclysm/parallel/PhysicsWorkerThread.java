package cataclysm.parallel;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;

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
	private ArrayBlockingQueue<PhysicsWork> queue = new ArrayBlockingQueue<PhysicsWork>(20);

	public PhysicsWorkerThread(int threadIndex, int threadCount, CyclicBarrier waitForTermination, CyclicBarrier waitForGroup) {
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
			w.run(this);
		}

	}

	void scheduleWork(PhysicsWork w) throws InterruptedException {
		queue.put(w);
	}

	public int getThreadIndex() {
		return threadIndex;
	}

	public void waitForTermination() {
		try {
			waitForTermination.await();
		} catch (InterruptedException | BrokenBarrierException e) {
			e.printStackTrace();
		}
	}

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

}
