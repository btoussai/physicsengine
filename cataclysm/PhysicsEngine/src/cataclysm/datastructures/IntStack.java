package cataclysm.datastructures;

/**
 * Defines a stack of ints.
 * @author Briac Toussaint
 *
 */
public class IntStack{
	private int[] elements;
	private int size = 0;
	
	/**
	 * Creates a stack with a default capacity of 10.
	 */
	public IntStack() {
		this(10);
	}
	
	public IntStack(int defaultCapacity) {
		elements = new int[defaultCapacity];
	}
	
	public void reserve(int capacity) {
		if(capacity > capacity()) {
			int[] elements = new int[capacity];
			System.arraycopy(this.elements, 0, elements, 0, this.elements.length);
			this.elements = elements;
		}
	}
	
	public void trimToSize() {
		if(size < capacity()) {
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
	
	public void push(int e) {
		if(size >= elements.length) {
			grow();
		}
		elements[size++] = e;
	}
	
	public int pop() {
		return elements[--size];
	}
	
	public boolean isEmpty() {
		return size==0;
	}
	
	public int size() {
		return size;
	}
	
	public int capacity() {
		return elements.length;
	}
}
