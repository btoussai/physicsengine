package cataclysm.record;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.function.Function;
import java.util.function.Supplier;

public final class ReadWriteList<T extends ReadWriteObject> implements ReadWriteObject, Iterable<T> {

	private final ArrayList<T> list = new ArrayList<T>();

	private final Function<RecordFile, T> readConstructor;
	private final Supplier<T> defaultConstructor;

	private int size;

	public ReadWriteList(Function<RecordFile, T> readConstructor, Supplier<T> defaultConstructor) {
		this.readConstructor = readConstructor;
		this.defaultConstructor = defaultConstructor;
	}

	public void add(T element) {
		list.add(element);
		size++;
	}

	public T get(int i) {
		if (i >= 0 && i < size) {
			return list.get(i);
		} else {
			throw new ArrayIndexOutOfBoundsException("Accessing element i=" + i + " size=" + size);
		}
	}

	public T getNext() {
		size++;
		if (list.size() >= size) {
			return list.get(size - 1);
		} else {
			T e = defaultConstructor.get();
			list.add(e);
			return e;
		}
	}

	@Override
	public void read(RecordFile f) {
		int size = f.readInt();
		list.ensureCapacity(size);

		int i = 0;
		int current_limit = Math.min(size, list.size());
		for (; i < current_limit; i++) {
			list.get(i).read(f);
		}

		for (; i < size; i++) {
			list.add(readConstructor.apply(f));
		}
		this.size = size;
	}

	@Override
	public void write(RecordFile f) {
		f.writeInt(size);
		for (int i = 0; i < size; i++) {
			list.get(i).write(f);
		}
	}

	@Override
	public Iterator<T> iterator() {
		return list.subList(0, size).iterator();
	}

	/**
	 * Sets the size to zero and deletes all elements
	 */
	public void clear() {
		list.clear();
		size = 0;
	}

	/**
	 * Sets the current size to zero but does not discard any elements for caching
	 * purposes
	 */
	public void rewind() {
		size = 0;
	}

	public int getElementCount() {
		return size;
	}

	@Override
	public int size() {
		int s = 0;
		for (int i = 0; i < size; i++)
			s += list.get(i).size();
		return 4 + s;
	}

}
