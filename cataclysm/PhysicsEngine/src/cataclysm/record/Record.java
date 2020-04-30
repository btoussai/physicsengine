package cataclysm.record;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;

public class Record {
	
	private final RecordFile file;
	private final List<Frame> frames = new ArrayList<Frame>();
	
	public Record(String path, boolean in) throws FileNotFoundException {
		file = new RecordFile(path, in);
	}

}
