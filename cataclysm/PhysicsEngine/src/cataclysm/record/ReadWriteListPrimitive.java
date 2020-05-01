package cataclysm.record;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.function.BiConsumer;
import java.util.function.Function;

public class ReadWriteListPrimitive<T> implements ReadWriteObject, Iterable<T> {

	private final ArrayList<T> list = new ArrayList<T>();
	private final Function<RecordFile, T> reader;
	private final BiConsumer<RecordFile, T> writer;
	private int size;
	private final int primitiveSize;

	public ReadWriteListPrimitive(Function<RecordFile, T> reader, BiConsumer<RecordFile, T> writer, int primitiveSize) {
		this.reader = reader;
		this.writer = writer;
		this.primitiveSize = primitiveSize;
	}
	
	public void add(T element) {
		list.add(element);
		size++;
	}

	@Override
	public void read(RecordFile f) {
		int size = f.readInt();
		list.ensureCapacity(size);

		int i = 0;
		int current_limit = Math.min(size, list.size());
		for (; i < current_limit; i++) {
			list.set(i, reader.apply(f));
		}

		for (; i < size; i++) {
			list.add(reader.apply(f));
		}
		this.size = size;
	}

	@Override
	public void write(RecordFile f) {
		f.writeInt(size);
		for (int i = 0; i < size; i++) {
			writer.accept(f, list.get(i));
		}
	}

	@Override
	public Iterator<T> iterator() {
		return list.subList(0, size).iterator();
	}

	public void clear() {
		list.clear();
		size = 0;
	}

	@Override
	public int size() {
		return 4 + size * primitiveSize;
	}

}
