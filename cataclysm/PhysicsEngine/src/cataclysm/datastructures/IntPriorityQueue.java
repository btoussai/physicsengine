package cataclysm.datastructures;

import java.util.Comparator;

import math.RandGen;

/**
 * Defines a priority queue of ints.
 * 
 * @author Briac Toussaint
 *
 */
public class IntPriorityQueue {
	private int[] elements;
	private int size = 0;
	private final Comparator<Integer> comp;

	/**
	 * Creates a priority queue with a default capacity of 10.
	 * 
	 * @param comp
	 */
	public IntPriorityQueue(Comparator<Integer> comp) {
		this(10, comp);
	}

	public IntPriorityQueue(int defaultCapacity, Comparator<Integer> comp) {
		elements = new int[defaultCapacity];
		this.comp = comp;
	}

	public void reserve(int capacity) {
		if (capacity > capacity()) {
			int[] elements = new int[capacity];
			System.arraycopy(this.elements, 0, elements, 0, this.elements.length);
			this.elements = elements;
		}
	}

	public void trimToSize() {
		if (size < capacity()) {
			int[] elements = new int[size];
			System.arraycopy(this.elements, 0, elements, 0, this.elements.length);
			this.elements = elements;
		}
	}

	private void grow() {
		int[] elements = new int[this.elements.length * 2 + 1];
		System.arraycopy(this.elements, 0, elements, 0, this.elements.length);
		this.elements = elements;
	}

	public void add(int e) {
		if (size >= elements.length) {
			grow();
		}
		siftUp(size, e, elements, comp);
		size++;
	}

	public int poll() {

		final int[] es;
		final int result = (es = elements)[0];

		final int n;
		final int x = es[(n = --size)];
		es[n] = 0;
		if (n > 0) {
			siftDown(0, x, n, es, comp);
		}
		return result;
	}

	public boolean isEmpty() {
		return size == 0;
	}

	public int size() {
		return size;
	}

	public int capacity() {
		return elements.length;
	}

	private static void siftUp(int k, int x, int[] es, Comparator<Integer> comp) {
		while (k > 0) {
			int parent = (k - 1) >>> 1;
			final int e = es[parent];
			if (comp.compare(x, e) >= 0)
				break;
			es[k] = e;
			k = parent;
		}
		es[k] = x;
	}

	private static void siftDown(int k, int x, int n, int[] es, Comparator<Integer> comp) {
		// assert n > 0;
		int half = n >>> 1;
		while (k < half) {
			int child = (k << 1) + 1;
			int c = es[child];
			int right = child + 1;
			if (right < n && comp.compare(c, es[right]) > 0)
				c = es[child = right];
			if (comp.compare(x, c) <= 0)
				break;
			es[k] = c;
			k = child;
		}
		es[k] = x;
	}
	
}
