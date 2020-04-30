package cataclysm.record;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.function.BiConsumer;
import java.util.function.Function;

public class ReadWriteListPrimitive<T> implements ReadWriteObject, Iterable<T> {

	private final ArrayList<T> list = new ArrayList<T>();
	private final Function<RecordFile, T> constructor;
	private final BiConsumer<RecordFile, T> writer;
	private int size;

	public ReadWriteListPrimitive(Function<RecordFile, T> constructor, BiConsumer<RecordFile, T> writer) {
		this.constructor = constructor;
		this.writer = writer;
	}

	@Override
	public void read(RecordFile f) {
		int size = f.readInt();
		list.ensureCapacity(size);

		int i = 0;
		int current_limit = Math.min(size, list.size());
		for (; i < current_limit; i++) {
			list.set(i, constructor.apply(f));
		}

		for (; i < size; i++) {
			list.add(constructor.apply(f));
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

}
